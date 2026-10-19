// SPDX-License-Identifier: GPL-3.0-only
//! Dark, low-contrast interface theme in the spirit of artist-focused CAD
//! tools: a near-black window frame, rounded cards on it, green toggles.

use egui::{Color32, CornerRadius, Stroke};

/// Toggles that are on, filled check marks and the active tool.
pub const ACCENT: Color32 = Color32::from_rgb(28, 158, 110);
/// The window frame behind the panels and around the viewport.
pub const FRAME: Color32 = Color32::from_rgb(10, 10, 11);
pub const PANEL: Color32 = Color32::from_rgb(28, 28, 31);
pub const PANEL_DARK: Color32 = Color32::from_rgb(20, 20, 22);
/// Cards in the side panels and the floating bars over the viewport.
pub const CARD: Color32 = Color32::from_rgb(30, 30, 33);
pub const FIELD: Color32 = Color32::from_rgb(42, 42, 46);
pub const HOVER: Color32 = Color32::from_rgb(56, 56, 61);
pub const TEXT: Color32 = Color32::from_rgb(214, 215, 219);
pub const TEXT_DIM: Color32 = Color32::from_rgb(136, 138, 145);
/// Selection in lists (the Outliner).
pub const SELECT: Color32 = Color32::from_rgb(62, 92, 150);

pub fn apply(ctx: &egui::Context) {
    let mut style = (*ctx.global_style()).clone();
    let v = &mut style.visuals;
    *v = egui::Visuals::dark();
    v.panel_fill = FRAME;
    v.window_fill = CARD;
    v.extreme_bg_color = FIELD;
    v.faint_bg_color = Color32::from_rgb(36, 36, 40);
    v.override_text_color = Some(TEXT);
    v.selection.bg_fill = SELECT;
    v.selection.stroke = Stroke::new(1.0, TEXT);
    v.hyperlink_color = ACCENT;
    v.window_stroke = Stroke::new(1.0, Color32::from_rgb(46, 46, 51));
    v.window_corner_radius = CornerRadius::same(8);
    v.menu_corner_radius = CornerRadius::same(6);
    for w in [&mut v.widgets.noninteractive, &mut v.widgets.inactive, &mut v.widgets.hovered, &mut v.widgets.active, &mut v.widgets.open] {
        w.corner_radius = CornerRadius::same(4);
    }
    v.widgets.noninteractive.bg_stroke = Stroke::new(1.0, Color32::from_rgb(40, 40, 44));
    v.widgets.inactive.weak_bg_fill = FIELD;
    v.widgets.inactive.bg_fill = FIELD;
    v.widgets.hovered.weak_bg_fill = HOVER;
    v.widgets.hovered.bg_fill = HOVER;
    v.widgets.hovered.bg_stroke = Stroke::NONE;
    v.widgets.active.weak_bg_fill = ACCENT.gamma_multiply(0.8);
    v.widgets.active.bg_fill = ACCENT.gamma_multiply(0.8);
    v.widgets.open.weak_bg_fill = HOVER;
    style.spacing.item_spacing = egui::vec2(6.0, 5.0);
    style.spacing.button_padding = egui::vec2(8.0, 4.0);
    style.spacing.interact_size.y = 22.0;
    ctx.set_global_style(style);
}

/// A green on/off switch, as on the side panel cards.
pub fn switch(ui: &mut egui::Ui, on: &mut bool) -> egui::Response {
    let size = egui::vec2(30.0, 16.0);
    let (rect, mut resp) = ui.allocate_exact_size(size, egui::Sense::click());
    if resp.clicked() {
        *on = !*on;
        resp.mark_changed();
    }
    let t = ui.ctx().animate_bool_responsive(resp.id, *on);
    let bg = if *on { ACCENT } else { FIELD };
    let p = ui.painter();
    p.rect_filled(rect, CornerRadius::same(8), bg);
    let x = egui::lerp(rect.left() + 8.0..=rect.right() - 8.0, t);
    p.circle_filled(egui::pos2(x, rect.center().y), 5.5, Color32::from_rgb(235, 236, 238));
    resp
}

/// A label on the left and a switch on the right, one card row.
pub fn switch_row(ui: &mut egui::Ui, label: &str, on: &mut bool) -> bool {
    let mut changed = false;
    ui.horizontal(|ui| {
        ui.label(label);
        ui.with_layout(egui::Layout::right_to_left(egui::Align::Center), |ui| {
            changed = switch(ui, on).changed();
        });
    });
    changed
}
