// SPDX-License-Identifier: GPL-3.0-only
// Reference images: textured quads placed in the scene.

struct Camera {
    view_proj: mat4x4<f32>,
    eye: vec4<f32>,
    viewport: vec4<f32>,
    forward: vec4<f32>,
    section: vec4<f32>,
    pull: vec4<f32>,
};
@group(0) @binding(0) var<uniform> cam: Camera;

struct RefImage {
    // Corners at image uv (0,0), (1,0), (1,1), (0,1); uv (0,0) is the
    // bottom left of the picture.
    c0: vec4<f32>,
    c1: vec4<f32>,
    c2: vec4<f32>,
    c3: vec4<f32>,
    // x: opacity.
    params: vec4<f32>,
};
@group(1) @binding(0) var<uniform> img: RefImage;
@group(1) @binding(1) var img_tex: texture_2d<f32>;
@group(1) @binding(2) var img_smp: sampler;

struct ImgOut {
    @builtin(position) clip: vec4<f32>,
    @location(0) uv: vec2<f32>,
};

@vertex
fn vs_image(@builtin(vertex_index) vi: u32) -> ImgOut {
    var uvs = array<vec2<f32>, 6>(vec2<f32>(0.0, 0.0), vec2<f32>(1.0, 0.0), vec2<f32>(1.0, 1.0), vec2<f32>(0.0, 0.0), vec2<f32>(1.0, 1.0), vec2<f32>(0.0, 1.0));
    let uv = uvs[vi];
    let p = mix(mix(img.c0.xyz, img.c1.xyz, uv.x), mix(img.c3.xyz, img.c2.xyz, uv.x), uv.y);
    var o: ImgOut;
    o.clip = cam.view_proj * vec4<f32>(p, 1.0);
    // Texture rows run top down.
    o.uv = vec2<f32>(uv.x, 1.0 - uv.y);
    return o;
}

@fragment
fn fs_image(i: ImgOut) -> @location(0) vec4<f32> {
    let c = textureSample(img_tex, img_smp, i.uv);
    return vec4<f32>(c.rgb, c.a * img.params.x);
}
