// SPDX-License-Identifier: AGPL-3.0-or-later

//! `navview` — a minimal wgpu viewer for the `rtx` bot navmesh. Renders a Quake BSP's world model as
//! untextured grey geometry and overlays the navmesh with one color per [`LinkKind`], the ballistic
//! link kinds drawn as their true arcs. Load a map by passing it as `argv[1]` or by dropping a `.bsp`
//! onto the window. A noclip-style fly camera moves with WASD + Space/C and looks with the right
//! mouse button held.

mod geom;
mod gpu;

use std::collections::HashSet;
use std::path::{Path, PathBuf};
use std::sync::Arc;
use std::time::Instant;

use glam::{Mat4, Vec3};
use rtx_nav::bsp::Bsp;
use rtx_nav::navmesh::{build_navmesh, NavBuild, NavGraph, RocketJumpParams, SpeedJumpParams};

use geom::NUM_LINK_KINDS;
use winit::application::ApplicationHandler;
use winit::event::{DeviceEvent, DeviceId, ElementState, MouseButton, WindowEvent};
use winit::event_loop::{ActiveEventLoop, ControlFlow, EventLoop, EventLoopProxy};
use winit::keyboard::{KeyCode, PhysicalKey};
use winit::window::{CursorGrabMode, Window, WindowId};

use gpu::Gpu;

/// Delivered from the background navmesh-build thread back to the event loop.
enum UserEvent {
    NavBuilt { generation: u64, result: NavBuild },
}

/// A noclip fly camera: a position plus yaw/pitch look angles (Quake Z-up, right-handed).
struct FlyCamera {
    pos: Vec3,
    yaw: f32,
    pitch: f32,
}

impl FlyCamera {
    fn dir(&self) -> Vec3 {
        let (cp, sp) = (self.pitch.cos(), self.pitch.sin());
        Vec3::new(cp * self.yaw.cos(), cp * self.yaw.sin(), sp)
    }

    fn view_proj(&self, aspect: f32) -> Mat4 {
        let proj = Mat4::perspective_rh(60f32.to_radians(), aspect.max(0.01), 4.0, 32768.0);
        proj * Mat4::look_to_rh(self.pos, self.dir(), Vec3::Z)
    }

    /// Frame the whole map: stand back from a high corner and look at the center.
    fn frame(&mut self, mins: Vec3, maxs: Vec3) {
        let center = (mins + maxs) * 0.5;
        let extent = (maxs - mins).length().max(64.0);
        self.pos = center + Vec3::new(0.9, 0.9, 0.7).normalize() * (extent * 0.6);
        let look = (center - self.pos).normalize_or(Vec3::NEG_X);
        self.yaw = look.y.atan2(look.x);
        self.pitch = look.z.clamp(-0.999, 0.999).asin();
    }
}

impl Default for FlyCamera {
    fn default() -> Self {
        FlyCamera { pos: Vec3::new(-256.0, 0.0, 128.0), yaw: 0.0, pitch: -0.3 }
    }
}

struct App {
    window: Option<Arc<Window>>,
    gpu: Option<Gpu>,
    camera: FlyCamera,
    keys: HashSet<KeyCode>,
    looking: bool,
    fast: bool,
    last_tick: Instant,
    proxy: EventLoopProxy<UserEvent>,
    generation: u64,
    pending_path: Option<PathBuf>,
    /// The most recently built navmesh, kept with its BSP so the overlay can be regenerated when a
    /// path-type toggle changes without rebuilding the graph (the BSP is needed to trim each cell's
    /// filled tile to its hull-1-supported footprint in [`geom::nav_surface`]).
    nav: Option<(Bsp, NavGraph)>,
    /// Per-`LinkKind` visibility (indexed by `geom::kind_index`); `Walk` gates the filled surface.
    visible: [bool; NUM_LINK_KINDS],
    egui_ctx: egui::Context,
    /// egui's winit input translator; created with the window in `resumed`.
    egui_state: Option<egui_winit::State>,
}

/// Base fly speed (units/sec); Shift multiplies it.
const MOVE_SPEED: f32 = 320.0;
const FAST_MULT: f32 = 4.0;
const LOOK_SENS: f32 = 0.003;
const PITCH_LIMIT: f32 = 1.55; // just under 90°

impl App {
    fn new(proxy: EventLoopProxy<UserEvent>, pending_path: Option<PathBuf>) -> Self {
        App {
            window: None,
            gpu: None,
            camera: FlyCamera::default(),
            keys: HashSet::new(),
            looking: false,
            fast: false,
            last_tick: Instant::now(),
            proxy,
            generation: 0,
            pending_path,
            nav: None,
            visible: [true; NUM_LINK_KINDS],
            egui_ctx: egui::Context::default(),
            egui_state: None,
        }
    }

    /// Regenerate and upload the navmesh overlay (filled walkable surface + colored link lines) from
    /// the current graph and path-type visibility. Cheap enough to redo on every toggle change.
    fn rebuild_overlay(&mut self) {
        let (Some(gpu), Some((bsp, graph))) = (self.gpu.as_mut(), self.nav.as_ref()) else { return };
        if self.visible[geom::kind_index(rtx_nav::navmesh::LinkKind::Walk)] {
            gpu.set_surface(&geom::nav_surface(graph, bsp));
        } else {
            gpu.set_surface(&[]);
        }
        gpu.set_lines(&geom::nav_lines(graph, &self.visible));
        if let Some(w) = &self.window {
            w.request_redraw();
        }
    }

    /// Run one egui frame and render the scene + UI. egui is cheap; a toggle change regenerates the
    /// overlay buffers before the draw so the change shows this frame.
    fn draw(&mut self) {
        let Some(window) = self.window.clone() else { return };
        if self.egui_state.is_none() || self.gpu.is_none() {
            return;
        }

        let raw_input = self.egui_state.as_mut().unwrap().take_egui_input(&window);
        let ctx = self.egui_ctx.clone();
        let mut visible = self.visible;
        let full = ctx.run_ui(raw_input, |ui| build_panel(ui, &mut visible));
        self.egui_state.as_mut().unwrap().handle_platform_output(&window, full.platform_output);

        if visible != self.visible {
            self.visible = visible;
            self.rebuild_overlay();
        }

        let ppp = full.pixels_per_point;
        let jobs = ctx.tessellate(full.shapes, ppp);
        let gpu = self.gpu.as_mut().unwrap();
        gpu.render(self.camera.view_proj(gpu.aspect()), &full.textures_delta, &jobs, ppp);
    }

    fn set_title(&self, text: &str) {
        if let Some(w) = &self.window {
            w.set_title(text);
        }
    }

    /// Load a BSP: show its grey geometry immediately, then build the navmesh on a worker thread.
    fn load(&mut self, path: &Path) {
        let Some(gpu) = self.gpu.as_mut() else { return };
        let name = path.file_name().map(|s| s.to_string_lossy().into_owned()).unwrap_or_default();

        let bytes = match std::fs::read(path) {
            Ok(b) => b,
            Err(e) => {
                self.set_title(&format!("navview — {name}: read error: {e}"));
                return;
            }
        };
        let Some(mesh) = geom::parse_render_mesh(&bytes) else {
            self.set_title(&format!("navview — {name}: not a supported BSP"));
            return;
        };

        gpu.set_mesh(&mesh.vertices);
        gpu.set_water(&mesh.water);
        gpu.clear_overlay();
        self.nav = None;
        self.camera.frame(mesh.mins, mesh.maxs);
        self.set_title(&format!("navview — {name} (building navmesh…)"));
        if let Some(w) = &self.window {
            w.request_redraw();
        }

        // Build the navmesh off-thread (a big map takes seconds with all solvers enabled). Standard
        // DM loadout: double-jump + speed-jump (bhop) + rocket-jump at stock physics; hooks off, and
        // plats/teleports/gates need live entities we don't have offline (empty vecs).
        self.generation += 1;
        let generation = self.generation;
        let proxy = self.proxy.clone();
        std::thread::spawn(move || {
            let result = build_navmesh(
                bytes,
                Vec::new(),
                Vec::new(),
                Vec::new(),
                None,
                true,
                Some(SpeedJumpParams {
                    gravity: 800.0,
                    accel: 10.0,
                    maxspeed: 320.0,
                    friction: 4.0,
                    stopspeed: 100.0,
                    curl: true,
                }),
                Some(RocketJumpParams { gravity: 800.0, rj_extra: 0.0 }),
            );
            let _ = proxy.send_event(UserEvent::NavBuilt { generation, result });
        });
    }

    /// Advance the fly camera by the movement keys currently held. Returns whether it moved.
    fn integrate(&mut self, dt: f32) -> bool {
        let mut delta = Vec3::ZERO;
        let dir = self.camera.dir();
        let right = dir.cross(Vec3::Z).normalize_or_zero();
        let mut add = |cond: bool, v: Vec3| {
            if cond {
                delta += v;
            }
        };
        add(self.keys.contains(&KeyCode::KeyW), dir);
        add(self.keys.contains(&KeyCode::KeyS), -dir);
        add(self.keys.contains(&KeyCode::KeyD), right);
        add(self.keys.contains(&KeyCode::KeyA), -right);
        add(self.keys.contains(&KeyCode::Space), Vec3::Z);
        add(self.keys.contains(&KeyCode::KeyC), -Vec3::Z);
        if delta == Vec3::ZERO {
            return false;
        }
        let speed = MOVE_SPEED * if self.fast { FAST_MULT } else { 1.0 };
        self.camera.pos += delta.normalize_or_zero() * speed * dt;
        true
    }

    fn set_looking(&mut self, on: bool) {
        self.looking = on;
        let Some(w) = &self.window else { return };
        w.set_cursor_visible(!on);
        if on {
            // Locked is ideal but unsupported on some platforms — fall back to Confined.
            let _ = w.set_cursor_grab(CursorGrabMode::Locked).or_else(|_| w.set_cursor_grab(CursorGrabMode::Confined));
        } else {
            let _ = w.set_cursor_grab(CursorGrabMode::None);
        }
    }
}

impl ApplicationHandler<UserEvent> for App {
    fn resumed(&mut self, el: &ActiveEventLoop) {
        if self.window.is_some() {
            return;
        }
        let attrs = Window::default_attributes().with_title("navview — drop a .bsp");
        let window = Arc::new(el.create_window(attrs).expect("create window"));
        self.gpu = Some(Gpu::new(window.clone()));
        self.egui_state = Some(egui_winit::State::new(
            self.egui_ctx.clone(),
            egui::ViewportId::ROOT,
            window.as_ref(),
            Some(window.scale_factor() as f32),
            None,
            None,
        ));
        window.request_redraw();
        self.window = Some(window);
        if let Some(path) = self.pending_path.take() {
            self.load(&path);
        }
    }

    fn window_event(&mut self, el: &ActiveEventLoop, _id: WindowId, event: WindowEvent) {
        // Let egui see the event first; if it consumed it (a click on the panel, typing in it),
        // don't also treat it as camera / hotkey input.
        let window = self.window.clone();
        if let (Some(window), Some(state)) = (window, self.egui_state.as_mut()) {
            let resp = state.on_window_event(&window, &event);
            if resp.repaint {
                window.request_redraw();
            }
            if resp.consumed {
                return;
            }
        }

        match event {
            WindowEvent::CloseRequested => el.exit(),
            WindowEvent::Resized(size) => {
                if let Some(gpu) = &mut self.gpu {
                    gpu.resize(size.width, size.height);
                }
            }
            WindowEvent::DroppedFile(path) => self.load(&path),
            WindowEvent::RedrawRequested => self.draw(),
            WindowEvent::MouseInput { state, button: MouseButton::Right, .. } => {
                self.set_looking(state == ElementState::Pressed);
            }
            WindowEvent::KeyboardInput { event, .. } => {
                if let PhysicalKey::Code(code) = event.physical_key {
                    let pressed = event.state == ElementState::Pressed;
                    if code == KeyCode::ShiftLeft || code == KeyCode::ShiftRight {
                        self.fast = pressed;
                    } else if code == KeyCode::Escape && pressed {
                        el.exit();
                    } else if pressed {
                        self.keys.insert(code);
                    } else {
                        self.keys.remove(&code);
                    }
                }
            }
            _ => {}
        }
    }

    fn device_event(&mut self, _el: &ActiveEventLoop, _id: DeviceId, event: DeviceEvent) {
        if self.looking {
            if let DeviceEvent::MouseMotion { delta: (dx, dy) } = event {
                self.camera.yaw -= dx as f32 * LOOK_SENS;
                self.camera.pitch = (self.camera.pitch - dy as f32 * LOOK_SENS).clamp(-PITCH_LIMIT, PITCH_LIMIT);
                if let Some(w) = &self.window {
                    w.request_redraw();
                }
            }
        }
    }

    fn user_event(&mut self, _el: &ActiveEventLoop, event: UserEvent) {
        let UserEvent::NavBuilt { generation, result } = event;
        if generation != self.generation {
            return; // a newer map was dropped while this build ran — discard the stale result
        }
        match result {
            Some((bsp, graph)) => {
                self.set_title(&format!(
                    "navview — {} cells, {} links",
                    graph.cells.len(),
                    graph.links.len()
                ));
                self.nav = Some((bsp, graph));
                self.rebuild_overlay();
            }
            None => self.set_title("navview — navmesh build failed"),
        }
        if let Some(w) = &self.window {
            w.request_redraw();
        }
    }

    fn about_to_wait(&mut self, el: &ActiveEventLoop) {
        let now = Instant::now();
        let dt = (now - self.last_tick).as_secs_f32().min(0.05); // clamp to avoid post-idle jumps
        self.last_tick = now;
        let moving = self.integrate(dt);
        if moving {
            if let Some(w) = &self.window {
                w.request_redraw();
            }
        }
        // Poll (drive continuous movement) only while a move key is held; otherwise idle in Wait.
        el.set_control_flow(if self.keys.is_empty() { ControlFlow::Wait } else { ControlFlow::Poll });
    }
}

/// The path-type toggle panel: a checkbox per `LinkKind`, labelled and swatched in that kind's
/// overlay color. `Walk` toggles the filled walkable surface; the rest toggle their colored lines.
fn build_panel(ui: &mut egui::Ui, visible: &mut [bool; NUM_LINK_KINDS]) {
    egui::Window::new("Path types")
        .default_pos([12.0, 12.0])
        .resizable(false)
        .show(ui.ctx(), |ui| {
            for kind in geom::LINK_KINDS {
                let [r, g, b] = geom::link_color(kind);
                let swatch = egui::Color32::from_rgb((r * 255.0) as u8, (g * 255.0) as u8, (b * 255.0) as u8);
                ui.horizontal(|ui| {
                    ui.checkbox(&mut visible[geom::kind_index(kind)], "");
                    ui.colored_label(swatch, geom::kind_label(kind));
                });
            }
        });
}

fn main() {
    let event_loop = EventLoop::<UserEvent>::with_user_event().build().expect("event loop");
    event_loop.set_control_flow(ControlFlow::Wait);
    let proxy = event_loop.create_proxy();
    let pending_path = std::env::args().nth(1).map(PathBuf::from);
    let mut app = App::new(proxy, pending_path);
    event_loop.run_app(&mut app).expect("run app");
}
