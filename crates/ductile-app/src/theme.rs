// SPDX-License-Identifier: GPL-3.0-only
//! Dark, low-contrast interface theme in the spirit of artist-focused CAD tools.

use egui::{Color32, CornerRadius, Stroke};

pub const ACCENT: Color32 = Color32::from_rgb(110, 140, 255);
pub const PANEL: Color32 = Color32::from_rgb(30, 31, 34);
pub const PANEL_DARK: Color32 = Color32::from_rgb(22, 23, 26);
pub const TEXT: Color32 = Color32::from_rgb(205, 207, 212);
pub const TEXT_DIM: Color32 = Color32::from_rgb(130, 133, 140);

pub fn apply(ctx: &egui::Context) {
    let mut style = (*ctx.global_style()).clone();
    let v = &mut style.visuals;
    *v = egui::Visuals::dark();
    v.panel_fill = PANEL;
    v.window_fill = PANEL;
    v.extreme_bg_color = PANEL_DARK;
    v.faint_bg_color = Color32::from_rgb(36, 37, 41);
    v.override_text_color = Some(TEXT);
    v.selection.bg_fill = ACCENT.gamma_multiply(0.55);
    v.selection.stroke = Stroke::new(1.0, ACCENT);
    v.hyperlink_color = ACCENT;
    v.window_stroke = Stroke::new(1.0, Color32::from_rgb(48, 50, 55));
    v.window_corner_radius = CornerRadius::same(8);
    for w in [&mut v.widgets.noninteractive, &mut v.widgets.inactive, &mut v.widgets.hovered, &mut v.widgets.active, &mut v.widgets.open] {
        w.corner_radius = CornerRadius::same(5);
    }
    v.widgets.noninteractive.bg_stroke = Stroke::new(1.0, Color32::from_rgb(44, 46, 51));
    v.widgets.inactive.weak_bg_fill = Color32::from_rgb(42, 44, 48);
    v.widgets.inactive.bg_fill = Color32::from_rgb(42, 44, 48);
    v.widgets.hovered.weak_bg_fill = Color32::from_rgb(54, 57, 63);
    v.widgets.hovered.bg_fill = Color32::from_rgb(54, 57, 63);
    v.widgets.active.weak_bg_fill = ACCENT.gamma_multiply(0.6);
    style.spacing.item_spacing = egui::vec2(6.0, 5.0);
    style.spacing.button_padding = egui::vec2(8.0, 4.0);
    ctx.set_global_style(style);
}
