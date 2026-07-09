// Two tiny pipelines sharing one camera uniform: grey lit triangles for the world model, and
// flat vertex-colored lines for the navmesh overlay.

struct Camera {
    view_proj: mat4x4<f32>,
};
@group(0) @binding(0) var<uniform> camera: Camera;

// --- mesh (world geometry) ---

struct MeshOut {
    @builtin(position) clip: vec4<f32>,
    @location(0) normal: vec3<f32>,
};

@vertex
fn vs_mesh(@location(0) pos: vec3<f32>, @location(1) normal: vec3<f32>) -> MeshOut {
    var out: MeshOut;
    out.clip = camera.view_proj * vec4<f32>(pos, 1.0);
    out.normal = normal;
    return out;
}

@fragment
fn fs_mesh(in: MeshOut) -> @location(0) vec4<f32> {
    let l = normalize(vec3<f32>(0.35, 0.5, 0.8));
    // abs() so both winding orders light the same — pairs with cull_mode = None.
    let shade = 0.30 + 0.55 * abs(dot(normalize(in.normal), l));
    return vec4<f32>(vec3<f32>(shade), 1.0);
}

// --- lines (navmesh overlay) ---

struct LineOut {
    @builtin(position) clip: vec4<f32>,
    @location(0) color: vec3<f32>,
};

@vertex
fn vs_line(@location(0) pos: vec3<f32>, @location(1) color: vec3<f32>) -> LineOut {
    var out: LineOut;
    out.clip = camera.view_proj * vec4<f32>(pos, 1.0);
    out.color = color;
    return out;
}

@fragment
fn fs_line(in: LineOut) -> @location(0) vec4<f32> {
    return vec4<f32>(in.color, 1.0);
}

// --- walkable surface (filled navmesh cell tiles) ---
// Shares vs_line (pos + color); drawn translucent so the map geometry shows through the overlay.

@fragment
fn fs_surf(in: LineOut) -> @location(0) vec4<f32> {
    return vec4<f32>(in.color, 0.45);
}

// --- liquid surfaces (water / lava / slime) ---
// Shares vs_line (pos + tint color); drawn with additive blending at 0.5 so liquids glow over the
// scene behind them.

@fragment
fn fs_water(in: LineOut) -> @location(0) vec4<f32> {
    return vec4<f32>(in.color, 0.5);
}
