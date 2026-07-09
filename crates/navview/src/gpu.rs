// SPDX-License-Identifier: AGPL-3.0-or-later

//! The wgpu backend: one surface, a depth buffer, and two pipelines (grey `TriangleList` for the
//! world model, vertex-colored `LineList` for the navmesh overlay) sharing a single camera uniform.
//! Geometry is re-uploaded wholesale on each map load — no streaming, this is a debug viewer.

use std::sync::Arc;

use glam::Mat4;
use wgpu::util::DeviceExt;
use winit::window::Window;

use crate::geom::{LineVertex, MeshVertex};

pub struct Gpu {
    surface: wgpu::Surface<'static>,
    device: wgpu::Device,
    queue: wgpu::Queue,
    config: wgpu::SurfaceConfiguration,
    depth_view: wgpu::TextureView,
    mesh_pipeline: wgpu::RenderPipeline,
    water_pipeline: wgpu::RenderPipeline,
    surf_pipeline: wgpu::RenderPipeline,
    line_pipeline: wgpu::RenderPipeline,
    camera_buf: wgpu::Buffer,
    camera_bind: wgpu::BindGroup,
    /// (buffer, vertex count) for the world mesh, the liquid surfaces, the walkable surface tiles,
    /// and the nav lines; `None` until a map is loaded / navmesh is built.
    mesh_vbuf: Option<(wgpu::Buffer, u32)>,
    water_vbuf: Option<(wgpu::Buffer, u32)>,
    surf_vbuf: Option<(wgpu::Buffer, u32)>,
    line_vbuf: Option<(wgpu::Buffer, u32)>,
    /// egui's wgpu backend, drawn in a second pass over the 3D scene each frame.
    egui_renderer: egui_wgpu::Renderer,
}

const DEPTH_FORMAT: wgpu::TextureFormat = wgpu::TextureFormat::Depth32Float;

/// Additive blend (`src·srcAlpha + dst`) for the translucent liquid surfaces — they brighten the
/// scene behind them rather than replacing it.
const ADDITIVE_BLEND: wgpu::BlendState = wgpu::BlendState {
    color: wgpu::BlendComponent {
        src_factor: wgpu::BlendFactor::SrcAlpha,
        dst_factor: wgpu::BlendFactor::One,
        operation: wgpu::BlendOperation::Add,
    },
    alpha: wgpu::BlendComponent {
        src_factor: wgpu::BlendFactor::One,
        dst_factor: wgpu::BlendFactor::One,
        operation: wgpu::BlendOperation::Add,
    },
};

impl Gpu {
    pub fn new(window: Arc<Window>) -> Gpu {
        let size = window.inner_size();
        let (w, h) = (size.width.max(1), size.height.max(1));

        let instance = wgpu::Instance::default();
        let surface = instance.create_surface(window).expect("create surface");
        let adapter = pollster::block_on(instance.request_adapter(&wgpu::RequestAdapterOptions {
            compatible_surface: Some(&surface),
            ..Default::default()
        }))
        .expect("no suitable GPU adapter");
        let (device, queue) = pollster::block_on(adapter.request_device(&wgpu::DeviceDescriptor {
            label: Some("navview device"),
            required_features: wgpu::Features::empty(),
            required_limits: wgpu::Limits::default(),
            ..Default::default()
        }))
        .expect("request device");

        let config = surface.get_default_config(&adapter, w, h).expect("surface config");
        surface.configure(&device, &config);
        let depth_view = make_depth(&device, w, h);

        // Camera uniform: a single 4x4 matrix, shared by both pipelines.
        let camera_buf = device.create_buffer(&wgpu::BufferDescriptor {
            label: Some("camera"),
            size: 64,
            usage: wgpu::BufferUsages::UNIFORM | wgpu::BufferUsages::COPY_DST,
            mapped_at_creation: false,
        });
        let bind_layout = device.create_bind_group_layout(&wgpu::BindGroupLayoutDescriptor {
            label: Some("camera layout"),
            entries: &[wgpu::BindGroupLayoutEntry {
                binding: 0,
                visibility: wgpu::ShaderStages::VERTEX,
                ty: wgpu::BindingType::Buffer {
                    ty: wgpu::BufferBindingType::Uniform,
                    has_dynamic_offset: false,
                    min_binding_size: None,
                },
                count: None,
            }],
        });
        let camera_bind = device.create_bind_group(&wgpu::BindGroupDescriptor {
            label: Some("camera bind"),
            layout: &bind_layout,
            entries: &[wgpu::BindGroupEntry {
                binding: 0,
                resource: camera_buf.as_entire_binding(),
            }],
        });

        let shader = device.create_shader_module(wgpu::ShaderModuleDescriptor {
            label: Some("navview shader"),
            source: wgpu::ShaderSource::Wgsl(include_str!("shader.wgsl").into()),
        });
        let layout = device.create_pipeline_layout(&wgpu::PipelineLayoutDescriptor {
            label: Some("navview layout"),
            bind_group_layouts: &[Some(&bind_layout)],
            immediate_size: 0,
        });

        let mesh_stride = std::mem::size_of::<MeshVertex>() as u64;
        let line_stride = std::mem::size_of::<LineVertex>() as u64;
        // Solid world geometry: opaque grey, lit, with backface culling — faces are wound so their
        // front (empty side) shows, so near shell walls drop out and you can see into the level.
        let mesh_pipeline = make_pipeline(
            &device,
            &layout,
            &shader,
            ("vs_mesh", "fs_mesh"),
            config.format,
            wgpu::PrimitiveTopology::TriangleList,
            wgpu::CompareFunction::Less,
            wgpu::BlendState::REPLACE,
            true,
            Some(wgpu::Face::Back),
            mesh_stride,
        );
        // Liquid surfaces: additive translucent, double-sided, depth-tested but not depth-writing.
        let water_pipeline = make_pipeline(
            &device,
            &layout,
            &shader,
            ("vs_line", "fs_water"),
            config.format,
            wgpu::PrimitiveTopology::TriangleList,
            wgpu::CompareFunction::Less,
            ADDITIVE_BLEND,
            false,
            None,
            line_stride,
        );
        // Translucent walkable-surface tiles: alpha-blended over the mesh, depth-tested but not
        // depth-writing, so overlapping tiles and the lines drawn afterward compose cleanly.
        let surf_pipeline = make_pipeline(
            &device,
            &layout,
            &shader,
            ("vs_line", "fs_surf"),
            config.format,
            wgpu::PrimitiveTopology::TriangleList,
            wgpu::CompareFunction::Less,
            wgpu::BlendState::ALPHA_BLENDING,
            false,
            None,
            line_stride,
        );
        let line_pipeline = make_pipeline(
            &device,
            &layout,
            &shader,
            ("vs_line", "fs_line"),
            config.format,
            wgpu::PrimitiveTopology::LineList,
            wgpu::CompareFunction::LessEqual,
            wgpu::BlendState::REPLACE,
            true,
            None,
            line_stride,
        );

        let egui_renderer = egui_wgpu::Renderer::new(&device, config.format, egui_wgpu::RendererOptions::default());

        Gpu {
            surface,
            device,
            queue,
            config,
            depth_view,
            mesh_pipeline,
            water_pipeline,
            surf_pipeline,
            line_pipeline,
            camera_buf,
            camera_bind,
            mesh_vbuf: None,
            water_vbuf: None,
            surf_vbuf: None,
            line_vbuf: None,
            egui_renderer,
        }
    }

    pub fn resize(&mut self, width: u32, height: u32) {
        if width == 0 || height == 0 {
            return; // minimized — keep the last valid config
        }
        self.config.width = width;
        self.config.height = height;
        self.surface.configure(&self.device, &self.config);
        self.depth_view = make_depth(&self.device, width, height);
    }

    pub fn aspect(&self) -> f32 {
        self.config.width as f32 / self.config.height as f32
    }

    /// Replace the world-mesh vertex buffer (grey triangles).
    pub fn set_mesh(&mut self, verts: &[MeshVertex]) {
        self.mesh_vbuf = self.upload(bytemuck::cast_slice(verts), verts.len() as u32, "mesh");
    }

    /// Replace the liquid-surface vertex buffer (additive translucent triangles).
    pub fn set_water(&mut self, verts: &[LineVertex]) {
        self.water_vbuf = self.upload(bytemuck::cast_slice(verts), verts.len() as u32, "water");
    }

    /// Replace the navmesh line overlay.
    pub fn set_lines(&mut self, verts: &[LineVertex]) {
        self.line_vbuf = self.upload(bytemuck::cast_slice(verts), verts.len() as u32, "lines");
    }

    /// Replace the translucent walkable-surface tiles.
    pub fn set_surface(&mut self, verts: &[LineVertex]) {
        self.surf_vbuf = self.upload(bytemuck::cast_slice(verts), verts.len() as u32, "surface");
    }

    /// Drop the whole navmesh overlay (surface + lines) — used while a new map's build is in flight.
    pub fn clear_overlay(&mut self) {
        self.line_vbuf = None;
        self.surf_vbuf = None;
    }

    fn upload(&self, data: &[u8], count: u32, label: &str) -> Option<(wgpu::Buffer, u32)> {
        if count == 0 {
            return None;
        }
        let buf = self.device.create_buffer_init(&wgpu::util::BufferInitDescriptor {
            label: Some(label),
            contents: data,
            usage: wgpu::BufferUsages::VERTEX,
        });
        Some((buf, count))
    }

    pub fn render(
        &mut self,
        view_proj: Mat4,
        textures_delta: &egui::TexturesDelta,
        paint_jobs: &[egui::ClippedPrimitive],
        pixels_per_point: f32,
    ) {
        self.queue
            .write_buffer(&self.camera_buf, 0, bytemuck::cast_slice(&view_proj.to_cols_array()));

        let frame = match self.surface.get_current_texture() {
            wgpu::CurrentSurfaceTexture::Success(f) | wgpu::CurrentSurfaceTexture::Suboptimal(f) => f,
            wgpu::CurrentSurfaceTexture::Outdated | wgpu::CurrentSurfaceTexture::Lost => {
                self.surface.configure(&self.device, &self.config);
                return;
            }
            _ => return, // Timeout / Occluded / Validation — skip this frame
        };
        let view = frame.texture.create_view(&wgpu::TextureViewDescriptor::default());
        let mut encoder = self
            .device
            .create_command_encoder(&wgpu::CommandEncoderDescriptor { label: Some("frame") });

        // Feed egui's textures and geometry to its renderer before the passes.
        for (id, delta) in &textures_delta.set {
            self.egui_renderer.update_texture(&self.device, &self.queue, *id, delta);
        }
        let screen = egui_wgpu::ScreenDescriptor {
            size_in_pixels: [self.config.width, self.config.height],
            pixels_per_point,
        };
        let egui_cmds = self
            .egui_renderer
            .update_buffers(&self.device, &self.queue, &mut encoder, paint_jobs, &screen);

        // 3D pass: clear, then draw the world mesh, the translucent walkable surface, and the links.
        {
            let mut pass = encoder.begin_render_pass(&wgpu::RenderPassDescriptor {
                label: Some("scene pass"),
                color_attachments: &[Some(wgpu::RenderPassColorAttachment {
                    view: &view,
                    resolve_target: None,
                    depth_slice: None,
                    ops: wgpu::Operations {
                        load: wgpu::LoadOp::Clear(wgpu::Color { r: 0.08, g: 0.09, b: 0.10, a: 1.0 }),
                        store: wgpu::StoreOp::Store,
                    },
                })],
                depth_stencil_attachment: Some(wgpu::RenderPassDepthStencilAttachment {
                    view: &self.depth_view,
                    depth_ops: Some(wgpu::Operations {
                        load: wgpu::LoadOp::Clear(1.0),
                        store: wgpu::StoreOp::Store,
                    }),
                    stencil_ops: None,
                }),
                timestamp_writes: None,
                occlusion_query_set: None,
                multiview_mask: None,
            });
            pass.set_bind_group(0, &self.camera_bind, &[]);
            if let Some((buf, count)) = &self.mesh_vbuf {
                pass.set_pipeline(&self.mesh_pipeline);
                pass.set_vertex_buffer(0, buf.slice(..));
                pass.draw(0..*count, 0..1);
            }
            // Additive liquid surfaces over the opaque geometry, then the translucent walkable
            // surface, then the opaque link lines on top.
            if let Some((buf, count)) = &self.water_vbuf {
                pass.set_pipeline(&self.water_pipeline);
                pass.set_vertex_buffer(0, buf.slice(..));
                pass.draw(0..*count, 0..1);
            }
            if let Some((buf, count)) = &self.surf_vbuf {
                pass.set_pipeline(&self.surf_pipeline);
                pass.set_vertex_buffer(0, buf.slice(..));
                pass.draw(0..*count, 0..1);
            }
            if let Some((buf, count)) = &self.line_vbuf {
                pass.set_pipeline(&self.line_pipeline);
                pass.set_vertex_buffer(0, buf.slice(..));
                pass.draw(0..*count, 0..1);
            }
        }

        // egui pass: load (preserve the scene), no depth. `render` wants a 'static pass.
        {
            let mut pass = encoder
                .begin_render_pass(&wgpu::RenderPassDescriptor {
                    label: Some("egui pass"),
                    color_attachments: &[Some(wgpu::RenderPassColorAttachment {
                        view: &view,
                        resolve_target: None,
                        depth_slice: None,
                        ops: wgpu::Operations { load: wgpu::LoadOp::Load, store: wgpu::StoreOp::Store },
                    })],
                    depth_stencil_attachment: None,
                    timestamp_writes: None,
                    occlusion_query_set: None,
                    multiview_mask: None,
                })
                .forget_lifetime();
            self.egui_renderer.render(&mut pass, paint_jobs, &screen);
        }
        for id in &textures_delta.free {
            self.egui_renderer.free_texture(id);
        }

        self.queue.submit(egui_cmds.into_iter().chain(std::iter::once(encoder.finish())));
        frame.present();
    }
}

fn make_depth(device: &wgpu::Device, w: u32, h: u32) -> wgpu::TextureView {
    device
        .create_texture(&wgpu::TextureDescriptor {
            label: Some("depth"),
            size: wgpu::Extent3d { width: w, height: h, depth_or_array_layers: 1 },
            mip_level_count: 1,
            sample_count: 1,
            dimension: wgpu::TextureDimension::D2,
            format: DEPTH_FORMAT,
            usage: wgpu::TextureUsages::RENDER_ATTACHMENT,
            view_formats: &[],
        })
        .create_view(&wgpu::TextureViewDescriptor::default())
}

#[allow(clippy::too_many_arguments)]
fn make_pipeline(
    device: &wgpu::Device,
    layout: &wgpu::PipelineLayout,
    shader: &wgpu::ShaderModule,
    (vs, fs): (&'static str, &'static str),
    format: wgpu::TextureFormat,
    topology: wgpu::PrimitiveTopology,
    depth_compare: wgpu::CompareFunction,
    blend: wgpu::BlendState,
    depth_write: bool,
    cull: Option<wgpu::Face>,
    stride: u64,
) -> wgpu::RenderPipeline {
    // Both vertex formats are two vec3s (pos, normal|color) → the same attribute layout.
    const ATTRS: [wgpu::VertexAttribute; 2] =
        wgpu::vertex_attr_array![0 => Float32x3, 1 => Float32x3];
    device.create_render_pipeline(&wgpu::RenderPipelineDescriptor {
        label: Some("navview pipeline"),
        layout: Some(layout),
        vertex: wgpu::VertexState {
            module: shader,
            entry_point: Some(vs),
            compilation_options: Default::default(),
            buffers: &[wgpu::VertexBufferLayout {
                array_stride: stride,
                step_mode: wgpu::VertexStepMode::Vertex,
                attributes: &ATTRS,
            }],
        },
        primitive: wgpu::PrimitiveState {
            topology,
            cull_mode: cull,
            ..Default::default()
        },
        depth_stencil: Some(wgpu::DepthStencilState {
            format: DEPTH_FORMAT,
            depth_write_enabled: Some(depth_write),
            depth_compare: Some(depth_compare),
            stencil: wgpu::StencilState::default(),
            bias: wgpu::DepthBiasState::default(),
        }),
        multisample: wgpu::MultisampleState::default(),
        fragment: Some(wgpu::FragmentState {
            module: shader,
            entry_point: Some(fs),
            compilation_options: Default::default(),
            targets: &[Some(wgpu::ColorTargetState {
                format,
                blend: Some(blend),
                write_mask: wgpu::ColorWrites::ALL,
            })],
        }),
        multiview_mask: None,
        cache: None,
    })
}
