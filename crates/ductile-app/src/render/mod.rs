// SPDX-License-Identifier: GPL-3.0-only
//! wgpu viewport renderer. Draws into an offscreen multisampled target which
//! egui then displays as an image.

use crate::camera::{Camera, Projection};
use bytemuck::{Pod, Zeroable};
use ductile_brep::BodyMesh;
use std::collections::HashMap;
use egui_wgpu::wgpu;
use wgpu::util::DeviceExt;

pub const MSAA: u32 = 4;
pub const COLOR_FORMAT: wgpu::TextureFormat = wgpu::TextureFormat::Rgba8UnormSrgb;
const DEPTH_FORMAT: wgpu::TextureFormat = wgpu::TextureFormat::Depth32Float;

#[repr(C)]
#[derive(Clone, Copy, Pod, Zeroable)]
struct CameraUniform {
    view_proj: [[f32; 4]; 4],
    eye: [f32; 4],
    viewport: [f32; 4],
    forward: [f32; 4],
    section: [f32; 4],
    pull: [f32; 4],
}

/// How faces are shaded for surface analysis.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub enum Analysis {
    #[default]
    None,
    /// Reflections of a striped room: continuity across joins.
    Zebra,
    /// Lines of equal angle to the view: smoothness of the surface flow.
    Isophote,
    /// Draft against the pull direction: green pulls, red is undercut,
    /// yellow is under the minimum angle.
    Draft,
    /// Front faces blue, back faces red.
    Direction,
    /// Materials lit by a studio environment.
    Render,
}

impl Analysis {
    pub const ALL: [Analysis; 6] = [Analysis::None, Analysis::Render, Analysis::Zebra, Analysis::Isophote, Analysis::Draft, Analysis::Direction];

    pub fn label(self) -> &'static str {
        match self {
            Analysis::None => "Shaded",
            Analysis::Zebra => "Zebra",
            Analysis::Isophote => "Isophote",
            Analysis::Draft => "Draft",
            Analysis::Direction => "Face direction",
            Analysis::Render => "Rendered",
        }
    }
}

#[repr(C)]
#[derive(Clone, Copy, Pod, Zeroable)]
struct ObjectUniform {
    color: [f32; 4],
    hover_color: [f32; 4],
    select_color: [f32; 4],
    flags: [u32; 4],
    /// x: metalness, y: roughness.
    pbr: [f32; 4],
}

#[repr(C)]
#[derive(Clone, Copy, Pod, Zeroable)]
struct MeshVertex {
    pos: [f32; 3],
    nrm: [f32; 3],
    face: u32,
}

/// One screen-space line segment (or a round point when `p0 == p1`).
#[repr(C)]
#[derive(Clone, Copy, Debug, Pod, Zeroable)]
pub struct LineInstance {
    pub p0: [f32; 3],
    pub p1: [f32; 3],
    pub color: [f32; 4],
    /// x: width px, y: unused, z: depth bias multiplier, w: unused.
    pub style: [f32; 4],
}

impl LineInstance {
    pub fn new(p0: glam::DVec3, p1: glam::DVec3, color: [f32; 4], width: f32) -> Self {
        Self { p0: p0.as_vec3().into(), p1: p1.as_vec3().into(), color, style: [width, 0.0, 1.0, 0.0] }
    }
    pub fn point(p: glam::DVec3, color: [f32; 4], size: f32) -> Self {
        Self { p0: p.as_vec3().into(), p1: p.as_vec3().into(), color, style: [size, 1.0, 2.0, 0.0] }
    }
}

/// Per-object display style.
#[derive(Clone, Debug)]
pub struct ObjectStyle {
    pub color: [f32; 4],
    pub hover_color: [f32; 4],
    pub select_color: [f32; 4],
    /// 0 = normal, 1 = hovered, 2 = selected (whole object).
    pub state: u32,
    pub visible: bool,
    /// Faces drawn (off: edges only).
    pub faces: bool,
    /// Drawn unlit and see-through after everything else (a curve's
    /// region fill).
    pub flat: bool,
    /// A closed solid: its inside shows as a cap where a section cuts it.
    pub solid: bool,
    pub metalness: f32,
    pub roughness: f32,
}

impl Default for ObjectStyle {
    fn default() -> Self {
        Self {
            color: [0.62, 0.63, 0.65, 1.0],
            hover_color: [0.74, 0.72, 0.62, 1.0],
            select_color: [0.95, 0.70, 0.25, 1.0],
            state: 0,
            visible: true,
            faces: true,
            flat: false,
            solid: false,
            metalness: 0.0,
            roughness: 0.45,
        }
    }
}

struct GpuObject {
    vbuf: wgpu::Buffer,
    ibuf: wgpu::Buffer,
    index_count: u32,
    obj_buf: wgpu::Buffer,
    face_buf: wgpu::Buffer,
    face_count: usize,
    bind_group: wgpu::BindGroup,
    edges: Option<(wgpu::Buffer, u32)>,
    visible: bool,
    faces: bool,
    flat: bool,
}

struct GpuImage {
    ubo: wgpu::Buffer,
    bind_group: wgpu::BindGroup,
    /// The pixels uploaded: identity of the shared buffer.
    key: usize,
}

/// A reference image as drawn: shared RGBA pixels, size, the four corners
/// (bottom left, bottom right, top right, top left) and opacity.
pub struct ImageDraw {
    pub id: u64,
    pub pixels: std::sync::Arc<Vec<u8>>,
    pub size: [u32; 2],
    pub corners: [glam::DVec3; 4],
    pub opacity: f32,
}

struct Targets {
    size: [u32; 2],
    msaa_view: wgpu::TextureView,
    depth_view: wgpu::TextureView,
    resolve_view: wgpu::TextureView,
    texture_id: egui::TextureId,
}

pub struct ViewportRenderer {
    device: wgpu::Device,
    queue: wgpu::Queue,
    cam_buf: wgpu::Buffer,
    cam_bg: wgpu::BindGroup,
    obj_bgl: wgpu::BindGroupLayout,
    mesh_pipeline: wgpu::RenderPipeline,
    mesh_xray_pipeline: wgpu::RenderPipeline,
    line_pipeline: wgpu::RenderPipeline,
    overlay_pipeline: wgpu::RenderPipeline,
    bg_pipeline: wgpu::RenderPipeline,
    image_pipeline: wgpu::RenderPipeline,
    image_bgl: wgpu::BindGroupLayout,
    sampler: wgpu::Sampler,
    images: Vec<(u64, GpuImage)>,
    targets: Option<Targets>,
    objects: HashMap<u64, GpuObject>,
    pub xray: bool,
    /// Surface analysis shading.
    pub analysis: Analysis,
    /// Pull direction and minimum angle (degrees) for draft analysis.
    pub pull: ([f32; 3], f32),
    /// Section analysis: the plane (normal, offset) beyond which the
    /// scene is cut away.
    pub section: Option<[f32; 4]>,
}

impl ViewportRenderer {
    pub fn new(device: wgpu::Device, queue: wgpu::Queue) -> Self {
        let shader = device.create_shader_module(wgpu::ShaderModuleDescriptor {
            label: Some("viewport shader"),
            source: wgpu::ShaderSource::Wgsl(include_str!("shader.wgsl").into()),
        });
        let cam_bgl = device.create_bind_group_layout(&wgpu::BindGroupLayoutDescriptor {
            label: Some("camera bgl"),
            entries: &[wgpu::BindGroupLayoutEntry {
                binding: 0,
                visibility: wgpu::ShaderStages::VERTEX_FRAGMENT,
                ty: wgpu::BindingType::Buffer {
                    ty: wgpu::BufferBindingType::Uniform,
                    has_dynamic_offset: false,
                    min_binding_size: None,
                },
                count: None,
            }],
        });
        let obj_bgl = device.create_bind_group_layout(&wgpu::BindGroupLayoutDescriptor {
            label: Some("object bgl"),
            entries: &[
                wgpu::BindGroupLayoutEntry {
                    binding: 0,
                    visibility: wgpu::ShaderStages::VERTEX_FRAGMENT,
                    ty: wgpu::BindingType::Buffer {
                        ty: wgpu::BufferBindingType::Uniform,
                        has_dynamic_offset: false,
                        min_binding_size: None,
                    },
                    count: None,
                },
                wgpu::BindGroupLayoutEntry {
                    binding: 1,
                    visibility: wgpu::ShaderStages::FRAGMENT,
                    ty: wgpu::BindingType::Buffer {
                        ty: wgpu::BufferBindingType::Storage { read_only: true },
                        has_dynamic_offset: false,
                        min_binding_size: None,
                    },
                    count: None,
                },
            ],
        });
        let cam_buf = device.create_buffer(&wgpu::BufferDescriptor {
            label: Some("camera ubo"),
            size: std::mem::size_of::<CameraUniform>() as u64,
            usage: wgpu::BufferUsages::UNIFORM | wgpu::BufferUsages::COPY_DST,
            mapped_at_creation: false,
        });
        let cam_bg = device.create_bind_group(&wgpu::BindGroupDescriptor {
            label: Some("camera bg"),
            layout: &cam_bgl,
            entries: &[wgpu::BindGroupEntry { binding: 0, resource: cam_buf.as_entire_binding() }],
        });
        let mesh_layout = device.create_pipeline_layout(&wgpu::PipelineLayoutDescriptor {
            label: Some("mesh layout"),
            bind_group_layouts: &[Some(&cam_bgl), Some(&obj_bgl)],
            immediate_size: 0,
        });
        let line_layout = device.create_pipeline_layout(&wgpu::PipelineLayoutDescriptor {
            label: Some("line layout"),
            bind_group_layouts: &[Some(&cam_bgl)],
            immediate_size: 0,
        });
        let ms = wgpu::MultisampleState { count: MSAA, mask: !0, alpha_to_coverage_enabled: false };
        let depth = |write: bool, compare: wgpu::CompareFunction| {
            Some(wgpu::DepthStencilState {
                format: DEPTH_FORMAT,
                depth_write_enabled: Some(write),
                depth_compare: Some(compare),
                stencil: Default::default(),
                bias: Default::default(),
            })
        };
        let mesh_attrs = wgpu::vertex_attr_array![0 => Float32x3, 1 => Float32x3, 2 => Uint32];
        let mesh_vbl = wgpu::VertexBufferLayout {
            array_stride: std::mem::size_of::<MeshVertex>() as u64,
            step_mode: wgpu::VertexStepMode::Vertex,
            attributes: &mesh_attrs,
        };
        let make_mesh = |label: &str, blend: Option<wgpu::BlendState>, write: bool| {
            device.create_render_pipeline(&wgpu::RenderPipelineDescriptor {
                label: Some(label),
                layout: Some(&mesh_layout),
                vertex: wgpu::VertexState {
                    module: &shader,
                    entry_point: Some("vs_mesh"),
                    buffers: &[Some(mesh_vbl.clone())],
                    compilation_options: Default::default(),
                },
                fragment: Some(wgpu::FragmentState {
                    module: &shader,
                    entry_point: Some("fs_mesh"),
                    targets: &[Some(wgpu::ColorTargetState {
                        format: COLOR_FORMAT,
                        blend,
                        write_mask: wgpu::ColorWrites::ALL,
                    })],
                    compilation_options: Default::default(),
                }),
                primitive: wgpu::PrimitiveState { cull_mode: None, ..Default::default() },
                depth_stencil: depth(write, wgpu::CompareFunction::Less),
                multisample: ms,
                multiview_mask: None,
                cache: None,
            })
        };
        let mesh_pipeline = make_mesh("mesh", None, true);
        let mesh_xray_pipeline = make_mesh("mesh xray", Some(wgpu::BlendState::ALPHA_BLENDING), false);
        let line_attrs = wgpu::vertex_attr_array![0 => Float32x3, 1 => Float32x3, 2 => Float32x4, 3 => Float32x4];
        let line_vbl = wgpu::VertexBufferLayout {
            array_stride: std::mem::size_of::<LineInstance>() as u64,
            step_mode: wgpu::VertexStepMode::Instance,
            attributes: &line_attrs,
        };
        let make_line = |label: &str, compare: wgpu::CompareFunction, fs: &str| {
            device.create_render_pipeline(&wgpu::RenderPipelineDescriptor {
                label: Some(label),
                layout: Some(&line_layout),
                vertex: wgpu::VertexState {
                    module: &shader,
                    entry_point: Some("vs_line"),
                    buffers: &[Some(line_vbl.clone())],
                    compilation_options: Default::default(),
                },
                fragment: Some(wgpu::FragmentState {
                    module: &shader,
                    entry_point: Some(fs),
                    targets: &[Some(wgpu::ColorTargetState {
                        format: COLOR_FORMAT,
                        blend: Some(wgpu::BlendState::ALPHA_BLENDING),
                        write_mask: wgpu::ColorWrites::ALL,
                    })],
                    compilation_options: Default::default(),
                }),
                primitive: wgpu::PrimitiveState { cull_mode: None, ..Default::default() },
                depth_stencil: depth(false, compare),
                multisample: ms,
                multiview_mask: None,
                cache: None,
            })
        };
        let line_pipeline = make_line("lines", wgpu::CompareFunction::LessEqual, "fs_line");
        let overlay_pipeline = make_line("overlay lines", wgpu::CompareFunction::Always, "fs_overlay");
        let bg_layout = device.create_pipeline_layout(&wgpu::PipelineLayoutDescriptor {
            label: Some("bg layout"),
            bind_group_layouts: &[],
            immediate_size: 0,
        });
        let bg_pipeline = device.create_render_pipeline(&wgpu::RenderPipelineDescriptor {
            label: Some("background"),
            layout: Some(&bg_layout),
            vertex: wgpu::VertexState {
                module: &shader,
                entry_point: Some("vs_bg"),
                buffers: &[],
                compilation_options: Default::default(),
            },
            fragment: Some(wgpu::FragmentState {
                module: &shader,
                entry_point: Some("fs_bg"),
                targets: &[Some(wgpu::ColorTargetState { format: COLOR_FORMAT, blend: None, write_mask: wgpu::ColorWrites::ALL })],
                compilation_options: Default::default(),
            }),
            primitive: Default::default(),
            depth_stencil: depth(false, wgpu::CompareFunction::Always),
            multisample: ms,
            multiview_mask: None,
            cache: None,
        });
        let image_shader = device.create_shader_module(wgpu::ShaderModuleDescriptor {
            label: Some("image shader"),
            source: wgpu::ShaderSource::Wgsl(include_str!("image.wgsl").into()),
        });
        let image_bgl = device.create_bind_group_layout(&wgpu::BindGroupLayoutDescriptor {
            label: Some("image bgl"),
            entries: &[
                wgpu::BindGroupLayoutEntry {
                    binding: 0,
                    visibility: wgpu::ShaderStages::VERTEX_FRAGMENT,
                    ty: wgpu::BindingType::Buffer { ty: wgpu::BufferBindingType::Uniform, has_dynamic_offset: false, min_binding_size: None },
                    count: None,
                },
                wgpu::BindGroupLayoutEntry {
                    binding: 1,
                    visibility: wgpu::ShaderStages::FRAGMENT,
                    ty: wgpu::BindingType::Texture { sample_type: wgpu::TextureSampleType::Float { filterable: true }, view_dimension: wgpu::TextureViewDimension::D2, multisampled: false },
                    count: None,
                },
                wgpu::BindGroupLayoutEntry { binding: 2, visibility: wgpu::ShaderStages::FRAGMENT, ty: wgpu::BindingType::Sampler(wgpu::SamplerBindingType::Filtering), count: None },
            ],
        });
        let image_layout = device.create_pipeline_layout(&wgpu::PipelineLayoutDescriptor {
            label: Some("image layout"),
            bind_group_layouts: &[Some(&cam_bgl), Some(&image_bgl)],
            immediate_size: 0,
        });
        let image_pipeline = device.create_render_pipeline(&wgpu::RenderPipelineDescriptor {
            label: Some("reference images"),
            layout: Some(&image_layout),
            vertex: wgpu::VertexState { module: &image_shader, entry_point: Some("vs_image"), buffers: &[], compilation_options: Default::default() },
            fragment: Some(wgpu::FragmentState {
                module: &image_shader,
                entry_point: Some("fs_image"),
                targets: &[Some(wgpu::ColorTargetState { format: COLOR_FORMAT, blend: Some(wgpu::BlendState::ALPHA_BLENDING), write_mask: wgpu::ColorWrites::ALL })],
                compilation_options: Default::default(),
            }),
            primitive: wgpu::PrimitiveState { cull_mode: None, ..Default::default() },
            depth_stencil: depth(false, wgpu::CompareFunction::LessEqual),
            multisample: ms,
            multiview_mask: None,
            cache: None,
        });
        let sampler = device.create_sampler(&wgpu::SamplerDescriptor {
            label: Some("image sampler"),
            mag_filter: wgpu::FilterMode::Linear,
            min_filter: wgpu::FilterMode::Linear,
            ..Default::default()
        });
        Self {
            image_pipeline,
            image_bgl,
            sampler,
            images: Vec::new(),
            device,
            queue,
            cam_buf,
            cam_bg,
            obj_bgl,
            mesh_pipeline,
            mesh_xray_pipeline,
            line_pipeline,
            overlay_pipeline,
            bg_pipeline,
            targets: None,
            objects: HashMap::new(),
            xray: false,
            analysis: Analysis::None,
            pull: ([0.0, 0.0, 1.0], 1.0),
            section: None,
        }
    }

    pub fn has_object(&self, id: u64) -> bool {
        self.objects.contains_key(&id)
    }

    /// Uploads (or replaces) the triangle mesh of an object.
    pub fn upload_object(&mut self, id: u64, mesh: &BodyMesh) {
        let mut verts = Vec::new();
        let mut idx: Vec<u32> = Vec::new();
        for (fi, f) in mesh.faces.iter().enumerate() {
            let base = verts.len() as u32;
            for (p, n) in f.positions.iter().zip(&f.normals) {
                verts.push(MeshVertex { pos: p.as_vec3().into(), nrm: n.as_vec3().into(), face: fi as u32 });
            }
            for t in &f.triangles {
                idx.extend(t.iter().map(|i| i + base));
            }
        }
        if verts.is_empty() {
            verts.push(MeshVertex { pos: [0.0; 3], nrm: [0.0, 0.0, 1.0], face: 0 });
        }
        if idx.is_empty() {
            idx.extend([0, 0, 0]);
        }
        let vbuf = self.device.create_buffer_init(&wgpu::util::BufferInitDescriptor {
            label: Some("mesh vbuf"),
            contents: bytemuck::cast_slice(&verts),
            usage: wgpu::BufferUsages::VERTEX,
        });
        let ibuf = self.device.create_buffer_init(&wgpu::util::BufferInitDescriptor {
            label: Some("mesh ibuf"),
            contents: bytemuck::cast_slice(&idx),
            usage: wgpu::BufferUsages::INDEX,
        });
        let face_count = mesh.faces.len().max(1);
        let obj_buf = self.device.create_buffer(&wgpu::BufferDescriptor {
            label: Some("object ubo"),
            size: std::mem::size_of::<ObjectUniform>() as u64,
            usage: wgpu::BufferUsages::UNIFORM | wgpu::BufferUsages::COPY_DST,
            mapped_at_creation: false,
        });
        let face_buf = self.device.create_buffer_init(&wgpu::util::BufferInitDescriptor {
            label: Some("face state"),
            contents: bytemuck::cast_slice(&vec![0u32; face_count]),
            usage: wgpu::BufferUsages::STORAGE | wgpu::BufferUsages::COPY_DST,
        });
        let bind_group = self.device.create_bind_group(&wgpu::BindGroupDescriptor {
            label: Some("object bg"),
            layout: &self.obj_bgl,
            entries: &[
                wgpu::BindGroupEntry { binding: 0, resource: obj_buf.as_entire_binding() },
                wgpu::BindGroupEntry { binding: 1, resource: face_buf.as_entire_binding() },
            ],
        });
        let edges = self.objects.remove(&id).and_then(|o| o.edges);
        self.objects.insert(
            id,
            GpuObject { vbuf, ibuf, index_count: idx.len() as u32, obj_buf, face_buf, face_count, bind_group, edges, visible: true, faces: true, flat: false },
        );
        self.set_style(id, &ObjectStyle::default(), &[]);
    }

    pub fn remove_object(&mut self, id: u64) {
        self.objects.remove(&id);
    }

    pub fn retain_objects(&mut self, keep: impl Fn(u64) -> bool) {
        self.objects.retain(|k, _| keep(*k));
    }

    pub fn set_style(&mut self, id: u64, style: &ObjectStyle, face_states: &[u32]) {
        let Some(o) = self.objects.get_mut(&id) else { return };
        let u = ObjectUniform {
            color: style.color,
            hover_color: style.hover_color,
            select_color: style.select_color,
            flags: [style.state, 0, style.solid as u32, style.flat as u32],
            pbr: [style.metalness, style.roughness, 0.0, 0.0],
        };
        self.queue.write_buffer(&o.obj_buf, 0, bytemuck::bytes_of(&u));
        let mut fs = vec![0u32; o.face_count];
        for (i, s) in face_states.iter().enumerate().take(o.face_count) {
            fs[i] = *s;
        }
        self.queue.write_buffer(&o.face_buf, 0, bytemuck::cast_slice(&fs));
        o.visible = style.visible;
        o.faces = style.faces;
        o.flat = style.flat;
    }

    pub fn set_edges(&mut self, id: u64, lines: &[LineInstance]) {
        let Some(o) = self.objects.get_mut(&id) else { return };
        if lines.is_empty() {
            o.edges = None;
            return;
        }
        let buf = self.device.create_buffer_init(&wgpu::util::BufferInitDescriptor {
            label: Some("edges"),
            contents: bytemuck::cast_slice(lines),
            usage: wgpu::BufferUsages::VERTEX,
        });
        o.edges = Some((buf, lines.len() as u32));
    }

    fn ensure_targets(&mut self, size: [u32; 2], egui_renderer: &mut egui_wgpu::Renderer) {
        if let Some(t) = &self.targets {
            if t.size == size {
                return;
            }
        }
        let mk = |label: &str, format: wgpu::TextureFormat, samples: u32, usage: wgpu::TextureUsages| {
            self.device
                .create_texture(&wgpu::TextureDescriptor {
                    label: Some(label),
                    size: wgpu::Extent3d { width: size[0], height: size[1], depth_or_array_layers: 1 },
                    mip_level_count: 1,
                    sample_count: samples,
                    dimension: wgpu::TextureDimension::D2,
                    format,
                    usage,
                    view_formats: &[],
                })
                .create_view(&Default::default())
        };
        let msaa_view = mk("viewport msaa", COLOR_FORMAT, MSAA, wgpu::TextureUsages::RENDER_ATTACHMENT);
        let depth_view = mk("viewport depth", DEPTH_FORMAT, MSAA, wgpu::TextureUsages::RENDER_ATTACHMENT);
        let resolve_view = mk(
            "viewport color",
            COLOR_FORMAT,
            1,
            wgpu::TextureUsages::RENDER_ATTACHMENT | wgpu::TextureUsages::TEXTURE_BINDING | wgpu::TextureUsages::COPY_SRC,
        );
        let texture_id = match self.targets.take() {
            Some(old) => {
                egui_renderer.update_egui_texture_from_wgpu_texture(
                    &self.device,
                    &resolve_view,
                    wgpu::FilterMode::Linear,
                    old.texture_id,
                );
                old.texture_id
            }
            None => egui_renderer.register_native_texture(&self.device, &resolve_view, wgpu::FilterMode::Linear),
        };
        self.targets = Some(Targets { size, msaa_view, depth_view, resolve_view, texture_id });
    }

    /// Brings the reference images up to date: new pixels are uploaded,
    /// placement and opacity rewritten, removed images dropped.
    pub fn set_images(&mut self, draws: &[ImageDraw]) {
        self.images.retain(|(id, _)| draws.iter().any(|d| d.id == *id));
        for d in draws {
            let key = std::sync::Arc::as_ptr(&d.pixels) as usize;
            let mut u = [0f32; 20];
            for (i, c) in d.corners.iter().enumerate() {
                u[i * 4..i * 4 + 3].copy_from_slice(&c.as_vec3().to_array());
                u[i * 4 + 3] = 1.0;
            }
            u[16] = d.opacity;
            if let Some((_, g)) = self.images.iter().find(|(id, g)| *id == d.id && g.key == key) {
                self.queue.write_buffer(&g.ubo, 0, bytemuck::cast_slice(&u));
                continue;
            }
            self.images.retain(|(id, _)| *id != d.id);
            let size = wgpu::Extent3d { width: d.size[0].max(1), height: d.size[1].max(1), depth_or_array_layers: 1 };
            let tex = self.device.create_texture(&wgpu::TextureDescriptor {
                label: Some("reference image"),
                size,
                mip_level_count: 1,
                sample_count: 1,
                dimension: wgpu::TextureDimension::D2,
                format: wgpu::TextureFormat::Rgba8UnormSrgb,
                usage: wgpu::TextureUsages::TEXTURE_BINDING | wgpu::TextureUsages::COPY_DST,
                view_formats: &[],
            });
            self.queue.write_texture(
                wgpu::TexelCopyTextureInfo { texture: &tex, mip_level: 0, origin: wgpu::Origin3d::ZERO, aspect: wgpu::TextureAspect::All },
                &d.pixels,
                wgpu::TexelCopyBufferLayout { offset: 0, bytes_per_row: Some(4 * size.width), rows_per_image: Some(size.height) },
                size,
            );
            let view = tex.create_view(&Default::default());
            let ubo = self.device.create_buffer_init(&wgpu::util::BufferInitDescriptor { label: Some("image ubo"), contents: bytemuck::cast_slice(&u), usage: wgpu::BufferUsages::UNIFORM | wgpu::BufferUsages::COPY_DST });
            let bind_group = self.device.create_bind_group(&wgpu::BindGroupDescriptor {
                label: Some("image bg"),
                layout: &self.image_bgl,
                entries: &[
                    wgpu::BindGroupEntry { binding: 0, resource: ubo.as_entire_binding() },
                    wgpu::BindGroupEntry { binding: 1, resource: wgpu::BindingResource::TextureView(&view) },
                    wgpu::BindGroupEntry { binding: 2, resource: wgpu::BindingResource::Sampler(&self.sampler) },
                ],
            });
            self.images.push((d.id, GpuImage { ubo, bind_group, key }));
        }
    }

    /// Renders the scene and returns the egui texture holding the result.
    pub fn render(
        &mut self,
        egui_renderer: &mut egui_wgpu::Renderer,
        camera: &Camera,
        size: [u32; 2],
        lines: &[LineInstance],
        overlay: &[LineInstance],
    ) -> egui::TextureId {
        let size = [size[0].max(1), size[1].max(1)];
        self.ensure_targets(size, egui_renderer);
        self.encode(camera, size, lines, overlay);
        self.targets.as_ref().unwrap().texture_id
    }

    fn encode(&mut self, camera: &Camera, size: [u32; 2], lines: &[LineInstance], overlay: &[LineInstance]) {
        let fwd = -camera.back();
        let cu = CameraUniform {
            view_proj: camera.view_proj_f32().to_cols_array_2d(),
            eye: camera.eye().as_vec3().extend(1.0).into(),
            viewport: [
                size[0] as f32,
                size[1] as f32,
                if camera.projection == Projection::Orthographic { 1.0 } else { 0.0 },
                0.0008,
            ],
            forward: fwd.as_vec3().extend(self.analysis as u32 as f32).into(),
            section: self.section.unwrap_or([0.0; 4]),
            pull: [self.pull.0[0], self.pull.0[1], self.pull.0[2], self.pull.1],
        };
        self.queue.write_buffer(&self.cam_buf, 0, bytemuck::bytes_of(&cu));
        let line_buf = (!lines.is_empty()).then(|| {
            self.device.create_buffer_init(&wgpu::util::BufferInitDescriptor {
                label: Some("frame lines"),
                contents: bytemuck::cast_slice(lines),
                usage: wgpu::BufferUsages::VERTEX,
            })
        });
        let overlay_buf = (!overlay.is_empty()).then(|| {
            self.device.create_buffer_init(&wgpu::util::BufferInitDescriptor {
                label: Some("overlay lines"),
                contents: bytemuck::cast_slice(overlay),
                usage: wgpu::BufferUsages::VERTEX,
            })
        });
        let t = self.targets.as_ref().unwrap();
        let mut enc = self.device.create_command_encoder(&Default::default());
        {
            let mut pass = enc.begin_render_pass(&wgpu::RenderPassDescriptor {
                label: Some("viewport"),
                color_attachments: &[Some(wgpu::RenderPassColorAttachment {
                    view: &t.msaa_view,
                    depth_slice: None,
                    resolve_target: Some(&t.resolve_view),
                    ops: wgpu::Operations { load: wgpu::LoadOp::Clear(wgpu::Color::BLACK), store: wgpu::StoreOp::Discard },
                })],
                depth_stencil_attachment: Some(wgpu::RenderPassDepthStencilAttachment {
                    view: &t.depth_view,
                    depth_ops: Some(wgpu::Operations { load: wgpu::LoadOp::Clear(1.0), store: wgpu::StoreOp::Discard }),
                    stencil_ops: None,
                }),
                timestamp_writes: None,
                occlusion_query_set: None,
                multiview_mask: None,
            });
            pass.set_pipeline(&self.bg_pipeline);
            pass.draw(0..3, 0..1);
            pass.set_bind_group(0, &self.cam_bg, &[]);
            pass.set_pipeline(if self.xray { &self.mesh_xray_pipeline } else { &self.mesh_pipeline });
            for o in self.objects.values().filter(|o| o.visible && o.faces && !o.flat) {
                pass.set_bind_group(1, &o.bind_group, &[]);
                pass.set_vertex_buffer(0, o.vbuf.slice(..));
                pass.set_index_buffer(o.ibuf.slice(..), wgpu::IndexFormat::Uint32);
                pass.draw_indexed(0..o.index_count, 0, 0..1);
            }
            // See-through fills last, blended over what is behind them.
            pass.set_pipeline(&self.mesh_xray_pipeline);
            for o in self.objects.values().filter(|o| o.visible && o.flat) {
                pass.set_bind_group(1, &o.bind_group, &[]);
                pass.set_vertex_buffer(0, o.vbuf.slice(..));
                pass.set_index_buffer(o.ibuf.slice(..), wgpu::IndexFormat::Uint32);
                pass.draw_indexed(0..o.index_count, 0, 0..1);
            }
            if !self.images.is_empty() {
                pass.set_pipeline(&self.image_pipeline);
                for (_, g) in &self.images {
                    pass.set_bind_group(1, &g.bind_group, &[]);
                    pass.draw(0..6, 0..1);
                }
            }
            pass.set_pipeline(&self.line_pipeline);
            for o in self.objects.values().filter(|o| o.visible) {
                if let Some((buf, n)) = &o.edges {
                    pass.set_vertex_buffer(0, buf.slice(..));
                    pass.draw(0..6, 0..*n);
                }
            }
            if let Some(b) = &line_buf {
                pass.set_vertex_buffer(0, b.slice(..));
                pass.draw(0..6, 0..lines.len() as u32);
            }
            if let Some(b) = &overlay_buf {
                pass.set_pipeline(&self.overlay_pipeline);
                pass.set_vertex_buffer(0, b.slice(..));
                pass.draw(0..6, 0..overlay.len() as u32);
            }
        }
        self.queue.submit([enc.finish()]);
    }
}
