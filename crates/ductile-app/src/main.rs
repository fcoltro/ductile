// SPDX-License-Identifier: GPL-3.0-only
//! Ductile — a free NURBS solid and surface modeler.
//!
//! Copyright (C) 2026 Ductile contributors.
//! This program is free software: you can redistribute it and/or modify it
//! under the terms of the GNU General Public License version 3.

mod app;
mod camera;
mod commands;
mod document;
mod picking;
mod render;
mod screenshot;
mod theme;
mod tools;
mod ui;

fn main() -> eframe::Result {
    env_logger::init();
    let mut opts = app::Options { screenshot: None, open: None, demo: false, camera: None };
    let mut args = std::env::args().skip(1);
    while let Some(a) = args.next() {
        match a.as_str() {
            "--screenshot" => opts.screenshot = args.next().map(Into::into),
            "--demo" => opts.demo = true,
            "--camera" => {
                // x,y,z,distance,yaw,pitch (degrees)
                let v: Vec<f64> = args.next().unwrap_or_default().split(',').filter_map(|x| x.trim().parse().ok()).collect();
                if v.len() == 6 {
                    opts.camera = Some([v[0], v[1], v[2], v[3], v[4], v[5]]);
                }
            }
            "--help" | "-h" => {
                println!("usage: ductile [FILE.ductile] [--demo] [--screenshot OUT.png] [--camera X,Y,Z,DIST,YAW,PITCH]");
                return Ok(());
            }
            other => opts.open = Some(other.into()),
        }
    }
    let native = eframe::NativeOptions {
        viewport: egui::ViewportBuilder::default().with_inner_size([1440.0, 900.0]).with_min_inner_size([800.0, 500.0]).with_title("Ductile"),
        renderer: eframe::Renderer::Wgpu,
        ..Default::default()
    };
    eframe::run_native("Ductile", native, Box::new(move |cc| Ok(Box::new(app::DuctileApp::new(cc, opts)))))
}
