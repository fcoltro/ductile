// SPDX-License-Identifier: GPL-3.0-only
//! Ductile — a free NURBS solid and surface modeler.
//!
//! Copyright (C) 2026 Ductile contributors.
//! This program is free software: you can redistribute it and/or modify it
//! under the terms of the GNU General Public License version 3.

mod analysis;
mod app;
mod backup;
mod background;
mod camera;
mod commands;
mod cplane;
mod expr;
mod keymap;
mod measure;
mod document;
mod icons;
mod picking;
mod prefs;
mod recent;
mod refimage;
mod select;
mod render;
mod text;
mod screenshot;
mod snap;
mod theme;
mod tools;
mod ui;
mod units;

fn main() -> eframe::Result {
    env_logger::init();
    let mut opts = app::Options { screenshot: None, open: None, demo: false, analysis: None, look: None, curvature: false, section: false, camera: None, select: None };
    let mut export_step: Option<std::path::PathBuf> = None;
    let mut export_lines: Option<std::path::PathBuf> = None;
    let mut args = std::env::args().skip(1);
    while let Some(a) = args.next() {
        match a.as_str() {
            "--screenshot" => opts.screenshot = args.next().map(Into::into),
            "--demo" => opts.demo = true,
            "--zebra" => opts.analysis = Some(render::Analysis::Zebra),
            "--isophote" => opts.analysis = Some(render::Analysis::Isophote),
            "--draft" => opts.analysis = Some(render::Analysis::Draft),
            "--direction" => opts.analysis = Some(render::Analysis::Direction),
            "--render" => opts.analysis = Some(render::Analysis::Render),
            "--look" => opts.look = args.next().and_then(|n| render::Look::ALL.into_iter().find(|l| l.label().eq_ignore_ascii_case(&n))),
            "--curvature" => opts.curvature = true,
            "--section" => opts.section = true,
            "--select" => opts.select = args.next().and_then(|x| x.parse().ok()),
            "--export-step" => export_step = args.next().map(Into::into),
            "--export-hidden-line" => export_lines = args.next().map(Into::into),
            "--camera" => {
                // x,y,z,distance,yaw,pitch (degrees)
                let v: Vec<f64> = args.next().unwrap_or_default().split(',').filter_map(|x| x.trim().parse().ok()).collect();
                if v.len() == 6 {
                    opts.camera = Some([v[0], v[1], v[2], v[3], v[4], v[5]]);
                }
            }
            "--help" | "-h" => {
                println!("usage: ductile [FILE.ductile|FILE.step] [--demo] [--screenshot OUT.png] [--camera X,Y,Z,DIST,YAW,PITCH] [--render|--zebra|--isophote|--draft|--direction] [--curvature] [--section] [--select N] [--export-step OUT.step] [--export-hidden-line OUT.svg]");
                return Ok(());
            }
            other => opts.open = Some(other.into()),
        }
    }
    if let Some(out) = export_lines {
        // Headless line drawing from the given (or a framing) camera.
        let mut doc = document::Document::default();
        if opts.demo {
            commands::demo_scene(&mut doc);
        } else if let Some(p) = &opts.open {
            if let Err(e) = commands::load_into(&mut doc, p) {
                eprintln!("{e}");
                std::process::exit(1);
            }
        }
        let mut cam = camera::Camera { aspect: 1.6, ..Default::default() };
        if let Some(b) = doc.objects.iter().map(|o| o.body.bbox()).reduce(|a, b| a.union(&b)) {
            cam.frame(b.center(), b.diagonal() * 0.3);
        }
        if let Some(c) = opts.camera {
            cam.target = glam::DVec3::new(c[0], c[1], c[2]);
            cam.distance = c[3];
            cam.yaw = c[4].to_radians();
            cam.pitch = c[5].to_radians();
        }
        match commands::export_hidden_line(&doc, &cam, &out, true) {
            Ok(()) => println!("wrote {}", out.display()),
            Err(e) => {
                eprintln!("hidden line export failed: {e}");
                std::process::exit(1);
            }
        }
        return Ok(());
    }
    if let Some(out) = export_step {
        // Headless conversion: build the document and write it out.
        let mut doc = document::Document::default();
        if opts.demo {
            commands::demo_scene(&mut doc);
        } else if let Some(p) = &opts.open {
            if let Err(e) = commands::load_into(&mut doc, p) {
                eprintln!("{e}");
                std::process::exit(1);
            }
        }
        match commands::export_step(&doc, &out) {
            Ok(n) => println!("wrote {n} shape(s) to {}", out.display()),
            Err(e) => {
                eprintln!("STEP export failed: {e}");
                std::process::exit(1);
            }
        }
        return Ok(());
    }
    let native = eframe::NativeOptions {
        viewport: egui::ViewportBuilder::default()
            .with_inner_size([1440.0, 900.0])
            .with_min_inner_size([800.0, 500.0])
            .with_title("Ductile")
            // Matches the desktop entry, so Wayland docks show the right icon.
            .with_app_id("io.github.fcoltro.Ductile")
            .with_icon(std::sync::Arc::new(window_icon())),
        renderer: eframe::Renderer::Wgpu,
        ..Default::default()
    };
    eframe::run_native("Ductile", native, Box::new(move |cc| Ok(Box::new(app::DuctileApp::new(cc, opts)))))
}

/// The application icon, decoded from the PNG shipped with the desktop entry.
fn window_icon() -> egui::IconData {
    let bytes: &[u8] = include_bytes!("../../../packaging/linux/ductile.png");
    let decoded = (|| {
        let mut r = png::Decoder::new(std::io::Cursor::new(bytes)).read_info().ok()?;
        let mut buf = vec![0; r.output_buffer_size()?];
        let info = r.next_frame(&mut buf).ok()?;
        (info.color_type == png::ColorType::Rgba && info.bit_depth == png::BitDepth::Eight).then(|| egui::IconData { rgba: buf[..info.buffer_size()].to_vec(), width: info.width, height: info.height })
    })();
    decoded.unwrap_or_default()
}

#[cfg(test)]
mod tests {
    #[test]
    fn window_icon_decodes() {
        let icon = super::window_icon();
        assert_eq!((icon.width, icon.height), (256, 256));
        assert_eq!(icon.rgba.len(), 256 * 256 * 4);
    }
}
