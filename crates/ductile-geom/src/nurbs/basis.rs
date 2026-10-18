// SPDX-License-Identifier: GPL-3.0-only
//! B-spline basis functions (Cox–de Boor), following the standard
//! formulation found in "The NURBS Book" (Piegl & Tiller).

/// Finds the knot span index `i` such that `knots[i] <= u < knots[i+1]`,
/// clamped so that `u == knots[n+1]` maps to the last span.
/// `n` is the index of the last control point.
pub fn find_span(n: usize, p: usize, u: f64, knots: &[f64]) -> usize {
    if u >= knots[n + 1] {
        return n;
    }
    if u <= knots[p] {
        return p;
    }
    let (mut low, mut high) = (p, n + 1);
    let mut mid = (low + high) / 2;
    while u < knots[mid] || u >= knots[mid + 1] {
        if u < knots[mid] {
            high = mid;
        } else {
            low = mid;
        }
        mid = (low + high) / 2;
    }
    mid
}

/// Non-vanishing basis functions `N[span-p..=span]` at `u`.
pub fn basis_funs(span: usize, u: f64, p: usize, knots: &[f64]) -> Vec<f64> {
    let mut n = vec![0.0; p + 1];
    let mut left = vec![0.0; p + 1];
    let mut right = vec![0.0; p + 1];
    n[0] = 1.0;
    for j in 1..=p {
        left[j] = u - knots[span + 1 - j];
        right[j] = knots[span + j] - u;
        let mut saved = 0.0;
        for r in 0..j {
            let denom = right[r + 1] + left[j - r];
            let temp = if denom.abs() < 1e-300 { 0.0 } else { n[r] / denom };
            n[r] = saved + right[r + 1] * temp;
            saved = left[j - r] * temp;
        }
        n[j] = saved;
    }
    n
}

/// Basis functions and their derivatives up to order `nd`.
/// Result `ders[k][j]` is the k-th derivative of `N[span-p+j]`.
pub fn ders_basis_funs(span: usize, u: f64, p: usize, nd: usize, knots: &[f64]) -> Vec<Vec<f64>> {
    let mut ndu = vec![vec![0.0; p + 1]; p + 1];
    let mut left = vec![0.0; p + 1];
    let mut right = vec![0.0; p + 1];
    ndu[0][0] = 1.0;
    for j in 1..=p {
        left[j] = u - knots[span + 1 - j];
        right[j] = knots[span + j] - u;
        let mut saved = 0.0;
        for r in 0..j {
            ndu[j][r] = right[r + 1] + left[j - r];
            let temp = if ndu[j][r].abs() < 1e-300 { 0.0 } else { ndu[r][j - 1] / ndu[j][r] };
            ndu[r][j] = saved + right[r + 1] * temp;
            saved = left[j - r] * temp;
        }
        ndu[j][j] = saved;
    }
    let mut ders = vec![vec![0.0; p + 1]; nd + 1];
    for j in 0..=p {
        ders[0][j] = ndu[j][p];
    }
    let mut a = vec![vec![0.0; p + 1]; 2];
    for r in 0..=p as isize {
        let (mut s1, mut s2) = (0usize, 1usize);
        a[0][0] = 1.0;
        for k in 1..=nd as isize {
            let mut d = 0.0;
            let rk = r - k;
            let pk = p as isize - k;
            if r >= k {
                let den = ndu[(pk + 1) as usize][rk as usize];
                a[s2][0] = if den.abs() < 1e-300 { 0.0 } else { a[s1][0] / den };
                d = a[s2][0] * ndu[rk as usize][pk as usize];
            }
            let j1 = if rk >= -1 { 1 } else { -rk };
            let j2 = if r - 1 <= pk { k - 1 } else { p as isize - r };
            for j in j1..=j2 {
                let den = ndu[(pk + 1) as usize][(rk + j) as usize];
                a[s2][j as usize] =
                    if den.abs() < 1e-300 { 0.0 } else { (a[s1][j as usize] - a[s1][(j - 1) as usize]) / den };
                d += a[s2][j as usize] * ndu[(rk + j) as usize][pk as usize];
            }
            if r <= pk {
                let den = ndu[(pk + 1) as usize][r as usize];
                a[s2][k as usize] = if den.abs() < 1e-300 { 0.0 } else { -a[s1][(k - 1) as usize] / den };
                d += a[s2][k as usize] * ndu[r as usize][pk as usize];
            }
            ders[k as usize][r as usize] = d;
            std::mem::swap(&mut s1, &mut s2);
        }
    }
    let mut r = p as f64;
    for k in 1..=nd {
        for j in 0..=p {
            ders[k][j] *= r;
        }
        r *= (p - k) as f64;
    }
    ders
}

pub fn binomial(n: usize, k: usize) -> f64 {
    let mut r = 1.0;
    for i in 0..k {
        r = r * (n - i) as f64 / (i + 1) as f64;
    }
    r
}

/// Clamped uniform knot vector for `n_ctrl` control points of degree `p`.
pub fn uniform_clamped_knots(n_ctrl: usize, p: usize) -> Vec<f64> {
    let m = n_ctrl + p + 1;
    let inner = n_ctrl - p;
    (0..m)
        .map(|i| {
            if i <= p {
                0.0
            } else if i >= n_ctrl {
                1.0
            } else {
                (i - p) as f64 / inner as f64
            }
        })
        .collect()
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn partition_of_unity() {
        let knots = [0.0, 0.0, 0.0, 1.0, 2.0, 3.0, 4.0, 4.0, 5.0, 5.0, 5.0];
        let n = knots.len() - 2 - 2;
        for i in 0..=50 {
            let u = 5.0 * i as f64 / 50.0;
            let s = find_span(n, 2, u, &knots);
            let b = basis_funs(s, u, 2, &knots);
            assert!((b.iter().sum::<f64>() - 1.0).abs() < 1e-12);
            let d = ders_basis_funs(s, u, 2, 2, &knots);
            assert!((d[0].iter().sum::<f64>() - 1.0).abs() < 1e-12);
            assert!(d[1].iter().sum::<f64>().abs() < 1e-9);
        }
    }

    #[test]
    fn known_values() {
        // Example 2.3 of The NURBS Book: u = 5/2.
        let knots = [0.0, 0.0, 0.0, 1.0, 2.0, 3.0, 4.0, 4.0, 5.0, 5.0, 5.0];
        let s = find_span(7, 2, 2.5, &knots);
        assert_eq!(s, 4);
        let b = basis_funs(s, 2.5, 2, &knots);
        assert!((b[0] - 1.0 / 8.0).abs() < 1e-12);
        assert!((b[1] - 6.0 / 8.0).abs() < 1e-12);
        assert!((b[2] - 1.0 / 8.0).abs() < 1e-12);
    }
}
