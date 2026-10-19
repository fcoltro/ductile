// SPDX-License-Identifier: GPL-3.0-only
//! Reference images: pictures placed in the scene to model over.

use ductile_geom::{DVec3, Frame};
use std::sync::Arc;

#[derive(Clone)]
pub struct RefImage {
    pub id: u64,
    pub name: String,
    pub path: std::path::PathBuf,
    /// RGBA, rows top down.
    pub pixels: Arc<Vec<u8>>,
    pub size: [u32; 2],
    /// Centre and orientation: the picture lies in the frame's xy plane,
    /// facing along z.
    pub frame: Frame,
    pub width: f64,
    pub opacity: f32,
    pub visible: bool,
}

impl RefImage {
    pub fn height(&self) -> f64 {
        self.width * self.size[1] as f64 / self.size[0].max(1) as f64
    }

    /// Bottom left, bottom right, top right, top left.
    pub fn corners(&self) -> [DVec3; 4] {
        let (x, y) = (self.frame.x * self.width * 0.5, self.frame.y * self.height() * 0.5);
        let c = self.frame.origin;
        [c - x - y, c + x - y, c + x + y, c - x + y]
    }
}

/// Decodes a PNG or JPEG file to RGBA.
pub fn load_rgba(path: &std::path::Path) -> Result<(Vec<u8>, [u32; 2]), String> {
    let bytes = std::fs::read(path).map_err(|e| e.to_string())?;
    decode(&bytes)
}

pub fn decode(bytes: &[u8]) -> Result<(Vec<u8>, [u32; 2]), String> {
    if bytes.starts_with(&[0x89, b'P', b'N', b'G']) {
        let mut dec = png::Decoder::new(std::io::Cursor::new(bytes));
        dec.set_transformations(png::Transformations::EXPAND | png::Transformations::STRIP_16);
        let mut r = dec.read_info().map_err(|e| e.to_string())?;
        let mut buf = vec![0; r.output_buffer_size().ok_or("PNG too large")?];
        let info = r.next_frame(&mut buf).map_err(|e| e.to_string())?;
        let px = &buf[..info.buffer_size()];
        let rgba = match info.color_type {
            png::ColorType::Rgba => px.to_vec(),
            png::ColorType::Rgb => px.chunks(3).flat_map(|c| [c[0], c[1], c[2], 255]).collect(),
            png::ColorType::GrayscaleAlpha => px.chunks(2).flat_map(|c| [c[0], c[0], c[0], c[1]]).collect(),
            png::ColorType::Grayscale => px.iter().flat_map(|&g| [g, g, g, 255]).collect(),
            png::ColorType::Indexed => return Err("unexpanded indexed PNG".into()),
        };
        return Ok((rgba, [info.width, info.height]));
    }
    if bytes.starts_with(&[0xFF, 0xD8]) {
        let mut dec = jpeg_decoder::Decoder::new(std::io::Cursor::new(bytes));
        let px = dec.decode().map_err(|e| e.to_string())?;
        let info = dec.info().ok_or("JPEG without a header")?;
        let rgba = match info.pixel_format {
            jpeg_decoder::PixelFormat::RGB24 => px.chunks(3).flat_map(|c| [c[0], c[1], c[2], 255]).collect(),
            jpeg_decoder::PixelFormat::L8 => px.iter().flat_map(|&g| [g, g, g, 255]).collect(),
            jpeg_decoder::PixelFormat::L16 => px.chunks(2).flat_map(|c| [c[0], c[0], c[0], 255]).collect(),
            jpeg_decoder::PixelFormat::CMYK32 => px.chunks(4).flat_map(|c| {
                let k = 255 - c[3] as u32;
                let f = |v: u8| ((255 - v as u32) * k / 255) as u8;
                [f(c[0]), f(c[1]), f(c[2]), 255]
            }).collect(),
        };
        return Ok((rgba, [info.width as u32, info.height as u32]));
    }
    Err("not a PNG or JPEG image".into())
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn decodes_png_and_places_corners() {
        // A 2×1 RGB PNG written with the png crate.
        let mut out = Vec::new();
        {
            let mut enc = png::Encoder::new(&mut out, 2, 1);
            enc.set_color(png::ColorType::Rgb);
            enc.set_depth(png::BitDepth::Eight);
            enc.write_header().unwrap().write_image_data(&[255, 0, 0, 0, 0, 255]).unwrap();
        }
        let (px, size) = decode(&out).unwrap();
        assert_eq!(size, [2, 1]);
        assert_eq!(px, vec![255, 0, 0, 255, 0, 0, 255, 255]);
        let img = RefImage { id: 1, name: "x".into(), path: "x.png".into(), pixels: Arc::new(px), size, frame: Frame::WORLD, width: 4.0, opacity: 0.5, visible: true };
        assert_eq!(img.height(), 2.0);
        assert_eq!(img.corners()[0], DVec3::new(-2.0, -1.0, 0.0));
        assert_eq!(img.corners()[2], DVec3::new(2.0, 1.0, 0.0));
        assert!(decode(b"nope").is_err());
    }
}
