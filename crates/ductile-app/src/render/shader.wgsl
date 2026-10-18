// SPDX-License-Identifier: GPL-3.0-only
// Viewport shaders: shaded faces, anti-aliased thick lines and points.

struct Camera {
    view_proj: mat4x4<f32>,
    eye: vec4<f32>,
    // x, y: viewport size in pixels; z: 1 if orthographic; w: line depth bias.
    viewport: vec4<f32>,
    // Camera forward direction (from eye into the scene); w: the analysis
    // shading (0 none, 1 zebra, 2 isophote, 3 draft, 4 face direction).
    forward: vec4<f32>,
    // Section plane: points with dot(xyz, p) + w > 0 are cut away; off
    // when xyz is zero.
    section: vec4<f32>,
    // Draft analysis: pull direction, and the minimum angle in degrees.
    pull: vec4<f32>,
};
@group(0) @binding(0) var<uniform> cam: Camera;

struct Object {
    color: vec4<f32>,
    hover_color: vec4<f32>,
    select_color: vec4<f32>,
    // x: whole-object state (0 none, 1 hovered, 2 selected); y: x-ray alpha flag; z: 1 for a solid.
    flags: vec4<u32>,
};
@group(1) @binding(0) var<uniform> obj: Object;
@group(1) @binding(1) var<storage, read> face_state: array<u32>;

struct MeshIn {
    @location(0) pos: vec3<f32>,
    @location(1) nrm: vec3<f32>,
    @location(2) face: u32,
};

struct MeshOut {
    @builtin(position) clip: vec4<f32>,
    @location(0) world: vec3<f32>,
    @location(1) nrm: vec3<f32>,
    @location(2) @interpolate(flat) face: u32,
};

@vertex
fn vs_mesh(v: MeshIn) -> MeshOut {
    var o: MeshOut;
    o.clip = cam.view_proj * vec4<f32>(v.pos, 1.0);
    o.world = v.pos;
    o.nrm = v.nrm;
    o.face = v.face;
    return o;
}

fn shade(base: vec3<f32>, n_in: vec3<f32>, world: vec3<f32>, front: bool) -> vec3<f32> {
    var n = normalize(n_in);
    var view_dir: vec3<f32>;
    if (cam.viewport.z > 0.5) {
        view_dir = -cam.forward.xyz;
    } else {
        view_dir = normalize(cam.eye.xyz - world);
    }
    if (!front) {
        n = -n;
    }
    // Camera-relative key light from upper left, fill from below, hemisphere ambient.
    let right = normalize(cross(cam.forward.xyz, vec3<f32>(0.0, 0.0, 1.0)) + vec3<f32>(1e-5, 0.0, 0.0));
    let up = normalize(cross(right, cam.forward.xyz));
    let key = normalize(-cam.forward.xyz * 0.8 + up * 0.6 - right * 0.4);
    let fill = normalize(-cam.forward.xyz * 0.5 - up * 0.5 + right * 0.6);
    let ndk = max(dot(n, key), 0.0);
    let ndf = max(dot(n, fill), 0.0);
    let hemi = mix(0.30, 0.55, n.z * 0.5 + 0.5);
    let h = normalize(key + view_dir);
    let spec = pow(max(dot(n, h), 0.0), 24.0) * 0.16;
    let rim = pow(1.0 - max(dot(n, view_dir), 0.0), 3.0) * 0.18;
    var c = base * (hemi + ndk * 0.62 + ndf * 0.18) + vec3<f32>(spec + rim);
    if (!front) {
        // Back faces of open sheets are tinted so orientation is readable.
        c = c * vec3<f32>(0.75, 0.70, 0.95);
    }
    return c;
}

// Reflection of a room lined with horizontal light bars: the stripes stay
// continuous across G1 joins and keep their direction across G2 ones.
fn zebra(c: vec3<f32>, n_in: vec3<f32>, world: vec3<f32>, front: bool) -> vec3<f32> {
    var n = normalize(n_in);
    if (!front) {
        n = -n;
    }
    var view_dir: vec3<f32>;
    if (cam.viewport.z > 0.5) {
        view_dir = -cam.forward.xyz;
    } else {
        view_dir = normalize(cam.eye.xyz - world);
    }
    let r = reflect(-view_dir, n);
    let elevation = atan2(r.z, length(r.xy));
    let s = fract(elevation * 10.0 / 3.14159265);
    // Anti-aliased bar edges from the screen-space rate of change.
    let w = max(fwidth(s), 1e-4);
    let bar = smoothstep(0.5 - w, 0.5 + w, s) * (1.0 - smoothstep(1.0 - w, 1.0, s)) + (1.0 - smoothstep(0.0, w, s));
    return mix(vec3<f32>(0.06), vec3<f32>(0.96), clamp(bar, 0.0, 1.0)) * 0.85 + c * 0.15;
}

fn cut(world: vec3<f32>) -> bool {
    let on = dot(cam.section.xyz, cam.section.xyz) > 0.0;
    return on && dot(cam.section.xyz, world) + cam.section.w > 0.0;
}

// Isophotes: lines where the surface meets the view at the same angle,
// every 10°. They run smoothly over G2 surfaces and kink at G1 joins.
fn isophote(c: vec3<f32>, n_in: vec3<f32>, world: vec3<f32>, front: bool) -> vec3<f32> {
    var n = normalize(n_in);
    if (!front) {
        n = -n;
    }
    var view_dir: vec3<f32>;
    if (cam.viewport.z > 0.5) {
        view_dir = -cam.forward.xyz;
    } else {
        view_dir = normalize(cam.eye.xyz - world);
    }
    let a = acos(clamp(dot(n, view_dir), -1.0, 1.0)) * 18.0 / 3.14159265;
    let s = abs(fract(a + 0.5) - 0.5);
    let w = max(fwidth(a), 1e-4);
    let line = 1.0 - smoothstep(0.5 * w, 1.5 * w, s);
    let band = select(vec3<f32>(0.20, 0.55, 0.85), vec3<f32>(0.90, 0.75, 0.30), fract(a * 0.5) > 0.5);
    return mix(c * 0.6 + band * 0.4, vec3<f32>(0.04), line);
}

// Draft against the pull direction: green where the face pulls by at least
// the minimum angle, red where it is undercut, yellow between.
fn draft(c: vec3<f32>, n_in: vec3<f32>, front: bool) -> vec3<f32> {
    var n = normalize(n_in);
    if (!front) {
        n = -n;
    }
    let ang = degrees(asin(clamp(dot(n, normalize(cam.pull.xyz)), -1.0, 1.0)));
    let lim = cam.pull.w;
    var tint = vec3<f32>(0.95, 0.85, 0.20);
    if (ang >= lim) {
        tint = vec3<f32>(0.25, 0.80, 0.35);
    } else if (ang <= -lim) {
        tint = vec3<f32>(0.90, 0.25, 0.22);
    }
    let lum = dot(c, vec3<f32>(0.3, 0.5, 0.2));
    return tint * (0.45 + lum * 0.8);
}

@fragment
fn fs_mesh(o: MeshOut, @builtin(front_facing) front: bool) -> @location(0) vec4<f32> {
    if (cut(o.world)) {
        discard;
    }
    // Looking into a cut solid shows its inside: paint it as a flat cap.
    if (!front && obj.flags.z == 1u && dot(cam.section.xyz, cam.section.xyz) > 0.0) {
        return vec4<f32>(obj.color.rgb * 0.55 + vec3<f32>(0.25, 0.08, 0.08), obj.color.a);
    }
    var base = obj.color.rgb;
    let fs = face_state[o.face];
    if (obj.flags.x == 2u || fs == 2u) {
        base = obj.select_color.rgb;
    } else if (obj.flags.x == 1u || fs == 1u) {
        base = obj.hover_color.rgb;
    }
    var c = shade(base, o.nrm, o.world, front);
    let mode = u32(cam.forward.w + 0.5);
    if (mode == 1u) {
        c = zebra(c, o.nrm, o.world, front);
    } else if (mode == 2u) {
        c = isophote(c, o.nrm, o.world, front);
    } else if (mode == 3u) {
        c = draft(c, o.nrm, front);
    } else if (mode == 4u) {
        if (front) {
            c = shade(vec3<f32>(0.25, 0.45, 0.95), o.nrm, o.world, true);
        } else {
            c = shade(vec3<f32>(0.95, 0.25, 0.22), -o.nrm, o.world, true);
        }
    }
    return vec4<f32>(c, obj.color.a);
}

// ---------------------------------------------------------------- lines ----

struct LineIn {
    @location(0) p0: vec3<f32>,
    @location(1) p1: vec3<f32>,
    @location(2) color: vec4<f32>,
    // x: width in pixels; y: 1 = draw as round point; z: depth bias multiplier.
    @location(3) style: vec4<f32>,
};

struct LineOut {
    @builtin(position) clip: vec4<f32>,
    @location(0) color: vec4<f32>,
    // Pixel position relative to segment start, segment vector, half width.
    @location(1) rel: vec2<f32>,
    @location(2) @interpolate(flat) seg: vec2<f32>,
    @location(3) @interpolate(flat) halfw: f32,
    @location(4) world: vec3<f32>,
};

fn clip_near(a: vec4<f32>, b: vec4<f32>) -> vec4<f32> {
    // Moves `a` along the segment to the near plane (w = eps) if behind it.
    let eps = 1e-4;
    if (a.w >= eps) {
        return a;
    }
    let t = (eps - a.w) / (b.w - a.w);
    return mix(a, b, t);
}

@vertex
fn vs_line(@builtin(vertex_index) vi: u32, l: LineIn) -> LineOut {
    var c0 = cam.view_proj * vec4<f32>(l.p0, 1.0);
    var c1 = cam.view_proj * vec4<f32>(l.p1, 1.0);
    let a = clip_near(c0, c1);
    let b = clip_near(c1, c0);
    let size = cam.viewport.xy;
    let s0 = (a.xy / a.w * 0.5 + 0.5) * size;
    let s1 = (b.xy / b.w * 0.5 + 0.5) * size;
    let halfw = l.style.x * 0.5;
    let ext = halfw + 1.0;
    var d = s1 - s0;
    let len = length(d);
    var dir = vec2<f32>(1.0, 0.0);
    if (len > 1e-4) {
        dir = d / len;
    }
    let nrm = vec2<f32>(-dir.y, dir.x);
    // Quad corners: (along, across) in {-1, 1} with the segment endpoints.
    var corner: vec2<f32>;
    var use_end = false;
    switch (vi) {
        case 0u: { corner = vec2<f32>(-1.0, -1.0); }
        case 1u: { corner = vec2<f32>(1.0, -1.0); use_end = true; }
        case 2u: { corner = vec2<f32>(1.0, 1.0); use_end = true; }
        case 3u: { corner = vec2<f32>(-1.0, -1.0); }
        case 4u: { corner = vec2<f32>(1.0, 1.0); use_end = true; }
        default: { corner = vec2<f32>(-1.0, 1.0); }
    }
    var base = s0;
    var cl = a;
    if (use_end) {
        base = s1;
        cl = b;
    }
    let px = base + dir * corner.x * ext + nrm * corner.y * ext;
    var o: LineOut;
    let ndc = px / size * 2.0 - 1.0;
    var z = cl.z / cl.w;
    // Pull lines slightly towards the viewer so edges win over their faces.
    z = z - cam.viewport.w * l.style.z * (1.0 - z) - 0.00002 * l.style.z;
    o.clip = vec4<f32>(ndc * cl.w, z * cl.w, cl.w);
    o.color = l.color;
    o.rel = px - s0;
    o.seg = d;
    o.halfw = halfw;
    o.world = l.p0;
    if (use_end) {
        o.world = l.p1;
    }
    return o;
}

@fragment
fn fs_line(o: LineOut) -> @location(0) vec4<f32> {
    if (cut(o.world)) {
        discard;
    }
    return line_color(o);
}

@fragment
fn fs_overlay(o: LineOut) -> @location(0) vec4<f32> {
    return line_color(o);
}

fn line_color(o: LineOut) -> vec4<f32> {
    let l2 = dot(o.seg, o.seg);
    var t = 0.0;
    if (l2 > 1e-8) {
        t = clamp(dot(o.rel, o.seg) / l2, 0.0, 1.0);
    }
    let dist = length(o.rel - o.seg * t);
    let alpha = clamp(o.halfw + 0.5 - dist, 0.0, 1.0);
    if (alpha <= 0.0) {
        discard;
    }
    return vec4<f32>(o.color.rgb, o.color.a * alpha);
}

// ------------------------------------------------------------ background ----

struct BgOut {
    @builtin(position) clip: vec4<f32>,
    @location(0) uv: vec2<f32>,
};

@vertex
fn vs_bg(@builtin(vertex_index) vi: u32) -> BgOut {
    var p = array<vec2<f32>, 3>(vec2<f32>(-1.0, -1.0), vec2<f32>(3.0, -1.0), vec2<f32>(-1.0, 3.0));
    var o: BgOut;
    o.clip = vec4<f32>(p[vi], 1.0, 1.0);
    o.uv = p[vi] * 0.5 + 0.5;
    return o;
}

@fragment
fn fs_bg(o: BgOut) -> @location(0) vec4<f32> {
    let top = vec3<f32>(0.090, 0.095, 0.105);
    let bottom = vec3<f32>(0.032, 0.034, 0.040);
    return vec4<f32>(mix(bottom, top, o.uv.y), 1.0);
}
