// yack: the SMLib corpus's fetch step: checks the models, writes each as STEP, finds SMLib's picks.
//! `yack-corpus-fetch`, run by `cargo xtask corpus smlib --fetch` (never at grading time, which
//! uses yack's public API only). SMLib's models are NVIDIA usd-brep test files (Apache-2.0); the
//! reader and STEP writer are cadrs's (yack-cadrs, temporary: this crate goes with it in M4 unless
//! the reader is ported first).
//!
//! ```text
//! yack-corpus-fetch verify DATA MANIFEST
//!     every `sha256  size  path` line of MANIFEST matches DATA/path; prints `ok N` or the mismatches
//! yack-corpus-fetch convert MODEL[#n] OUT.step [--edges I,J,..] [--faces I,J,..]
//!     writes the model (or B-rep n of a part) as STEP and prints `edges N` and `faces N` (SMLib's
//!     counts) and `planar true|false` (every face a plane), then `edge I x y z` for each edge asked for: the point of the body's edge that
//!     SMLib's edge I became, at the middle of its parameter range, and `face I x y z`: a point
//!     on the body face SMLib's face I became (its area centroid projected onto its surface)
//! ```
//! A pick that no body edge or face matches, or that two match, is `refused <why>` on its line:
//! selection data the runner then reports as a refusal, never a guess.
use std::process::ExitCode;

use yack_cadrs::native::NativeKernel;
use yack_cadrs::native::math::P3;
use yack_cadrs::native::smb;
use yack_cadrs::native::topo::Body;

fn main() -> ExitCode {
    let args: Vec<String> = std::env::args().skip(1).collect();
    let r = match args.first().map(String::as_str) {
        Some("verify") => verify(&args),
        Some("convert") => convert(&args),
        _ => Err("usage: yack-corpus-fetch verify DATA MANIFEST | convert MODEL[#n] OUT.step [--edges ..] [--faces ..]".into()),
    };
    match r {
        Ok(()) => ExitCode::SUCCESS,
        Err(e) => {
            eprintln!("yack-corpus-fetch: {e}");
            ExitCode::FAILURE
        }
    }
}

fn arg(args: &[String], k: usize, what: &str) -> Result<String, String> {
    args.get(k).cloned().ok_or(format!("missing {what}"))
}

fn verify(args: &[String]) -> Result<(), String> {
    let (data, manifest) = (arg(args, 1, "DATA")?, arg(args, 2, "MANIFEST")?);
    let text = std::fs::read_to_string(&manifest).map_err(|e| format!("{manifest}: {e}"))?;
    let (mut ok, mut bad) = (0usize, Vec::new());
    for line in text.lines().filter(|l| !l.starts_with('#') && !l.trim().is_empty()) {
        let parts: Vec<&str> = line.split_whitespace().collect();
        let [want, size, rel] = parts.as_slice() else { return Err(format!("bad manifest line: {line}")) };
        match std::fs::read(format!("{data}/{rel}")) {
            Ok(bytes) if bytes.len().to_string() == *size && sha256_hex(&bytes) == *want => ok += 1,
            Ok(_) => bad.push(format!("{rel}: differs")),
            Err(e) => bad.push(format!("{rel}: {e}")),
        }
    }
    println!("ok {ok}");
    for b in &bad {
        println!("bad {b}");
    }
    if bad.is_empty() { Ok(()) } else { Err(format!("{} files differ from the manifest", bad.len())) }
}

fn indices(args: &[String], flag: &str) -> Result<Vec<usize>, String> {
    let Some(k) = args.iter().position(|a| a == flag) else { return Ok(Vec::new()) };
    let list = arg(args, k + 1, flag)?;
    list.split(',').filter(|s| !s.is_empty()).map(|s| s.parse::<usize>().map_err(|e| format!("{flag} {s}: {e}"))).collect()
}

fn convert(args: &[String]) -> Result<(), String> {
    let (model, out) = (arg(args, 1, "MODEL")?, arg(args, 2, "OUT.step")?);
    let (file, n) = match model.split_once('#') {
        Some((f, b)) => (f.to_string(), b.parse::<usize>().map_err(|e| e.to_string())?),
        None => (model.clone(), 0),
    };
    let bytes = std::fs::read(&file).map_err(|e| format!("{file}: {e}"))?;
    let brep = if file.ends_with(".smp") {
        let mut p = smb::read_part(&bytes).map_err(|e| format!("{file}: {e}"))?;
        if n >= p.breps.len() {
            return Err(format!("{file} has {} B-reps", p.breps.len()));
        }
        p.breps.swap_remove(n)
    } else {
        smb::read_brep(&bytes).map_err(|e| format!("{file}: {e}"))?
    };
    let mut k = NativeKernel::new();
    let id = k.insert(brep.body.clone());
    let step = yack_cadrs::native::exchange::export_step(&k, &[id]).map_err(|e| format!("STEP: {e:?}"))?;
    std::fs::write(&out, step).map_err(|e| format!("{out}: {e}"))?;
    println!("edges {}", brep.edge_mids.len());
    println!("faces {}", brep.face_centroids.len());
    // counts are compared only when every face is planar (cadrs's smlib_common::check_result)
    let planar = brep.body.faces.iter().all(|f| matches!(f.surf, yack_cadrs::native::surface::Surface::Plane { .. }));
    println!("planar {planar}");
    let body = &brep.body;
    let size = { let d = body.bbox().diagonal(); if d > 0.0 { d } else { 1.0 } };
    for i in indices(args, "--edges")? {
        match brep.edge_mids.get(i).map(|m| edge_point(body, m, size)) {
            Some(Ok(p)) => println!("edge {i} {} {} {}", p.x, p.y, p.z),
            Some(Err(why)) => println!("edge {i} refused {why}"),
            None => println!("edge {i} refused SMLib has no edge {i}"),
        }
    }
    for i in indices(args, "--faces")? {
        match brep.face_centroids.get(i).copied().flatten().map(|c| face_point(body, &c, size)) {
            Some(Ok(p)) => println!("face {i} {} {} {}", p.x, p.y, p.z),
            Some(Err(why)) => println!("face {i} refused {why}"),
            None => println!("face {i} refused SMLib's face {i} has no body face"),
        }
    }
    Ok(())
}

/// The point at the middle of the body edge passing through `m` (cadrs's `pick_edges` test:
/// within 1e-4 of the model's size, and no second edge within 1e-7 of it).
fn edge_point(body: &Body, m: &P3, size: f64) -> Result<P3, String> {
    let mut d: Vec<(f64, usize)> = body
        .edges
        .iter()
        .enumerate()
        .filter(|(_, e)| !e.degenerate)
        .map(|(i, e)| ((e.point(e.curve.closest(m, e.t[0], e.t[1])) - m).norm(), i))
        .collect();
    d.sort_by(|a, b| a.0.total_cmp(&b.0));
    let Some(&(d1, best)) = d.first() else { return Err("no edges".into()) };
    if d1 > 1e-4 * size { // tol: cadrs's pick_edges test, 1e-4 of the model's size
        return Err(format!("no edge passes through it ({d1:.3e} off)"));
    }
    if d.get(1).is_some_and(|&(d2, _)| d2 <= d1 + 1e-7 * size) { // tol: cadrs's pick_edges test
        return Err("two edges pass through it".into());
    }
    let e = body.edges.get(best).ok_or("no such edge")?;
    Ok(e.point(e.mid_t()))
}

/// A point on the body face whose area centroid is nearest `c` (cadrs's `pick_faces` test), that
/// centroid projected onto the face's surface.
fn face_point(body: &Body, c: &P3, size: f64) -> Result<P3, String> {
    let cents = smb::face_centroids(body);
    let mut d: Vec<(f64, usize)> = cents.iter().enumerate().map(|(i, x)| ((x - c).norm(), i)).collect();
    d.sort_by(|a, b| a.0.total_cmp(&b.0));
    let Some(&(d1, best)) = d.first() else { return Err("no faces".into()) };
    if d1 > 1e-4 * size { // tol: cadrs's pick_faces test, 1e-4 of the model's size
        return Err(format!("no face's centroid is there ({d1:.3e} off)"));
    }
    if d.get(1).is_some_and(|&(d2, _)| d2 <= d1 + 1e-7 * size) { // tol: cadrs's pick_faces test
        return Err("two faces share that centroid".into());
    }
    let f = body.faces.get(best).ok_or("no such face")?;
    let (u, v) = f.surf.project(c, None);
    Ok(f.surf.eval(u, v))
}

/// SHA-256 (FIPS 180-4) as lower-case hex: the manifest's check needs no dependency.
fn sha256_hex(data: &[u8]) -> String {
    const K: [u32; 64] = [
        0x428a2f98, 0x71374491, 0xb5c0fbcf, 0xe9b5dba5, 0x3956c25b, 0x59f111f1, 0x923f82a4, 0xab1c5ed5, 0xd807aa98, 0x12835b01, 0x243185be,
        0x550c7dc3, 0x72be5d74, 0x80deb1fe, 0x9bdc06a7, 0xc19bf174, 0xe49b69c1, 0xefbe4786, 0x0fc19dc6, 0x240ca1cc, 0x2de92c6f, 0x4a7484aa,
        0x5cb0a9dc, 0x76f988da, 0x983e5152, 0xa831c66d, 0xb00327c8, 0xbf597fc7, 0xc6e00bf3, 0xd5a79147, 0x06ca6351, 0x14292967, 0x27b70a85,
        0x2e1b2138, 0x4d2c6dfc, 0x53380d13, 0x650a7354, 0x766a0abb, 0x81c2c92e, 0x92722c85, 0xa2bfe8a1, 0xa81a664b, 0xc24b8b70, 0xc76c51a3,
        0xd192e819, 0xd6990624, 0xf40e3585, 0x106aa070, 0x19a4c116, 0x1e376c08, 0x2748774c, 0x34b0bcb5, 0x391c0cb3, 0x4ed8aa4a, 0x5b9cca4f,
        0x682e6ff3, 0x748f82ee, 0x78a5636f, 0x84c87814, 0x8cc70208, 0x90befffa, 0xa4506ceb, 0xbef9a3f7, 0xc67178f2,
    ];
    let mut h: [u32; 8] = [0x6a09e667, 0xbb67ae85, 0x3c6ef372, 0xa54ff53a, 0x510e527f, 0x9b05688c, 0x1f83d9ab, 0x5be0cd19];
    let mut msg = data.to_vec();
    let bits = (data.len() as u64).wrapping_mul(8);
    msg.push(0x80);
    while msg.len() % 64 != 56 {
        msg.push(0);
    }
    msg.extend_from_slice(&bits.to_be_bytes());
    let (blocks, _) = msg.as_chunks::<64>();
    for block in blocks {
        let mut w = [0u32; 64];
        let (words, _) = block.as_chunks::<4>();
        for (wi, c) in w.iter_mut().zip(words) {
            *wi = u32::from_be_bytes(*c);
        }
        for i in 16..64 {
            let at = |k: usize| w.get(k).copied().unwrap_or(0);
            let s0 = at(i - 15).rotate_right(7) ^ at(i - 15).rotate_right(18) ^ (at(i - 15) >> 3);
            let s1 = at(i - 2).rotate_right(17) ^ at(i - 2).rotate_right(19) ^ (at(i - 2) >> 10);
            let v = at(i - 16).wrapping_add(s0).wrapping_add(at(i - 7)).wrapping_add(s1);
            if let Some(x) = w.get_mut(i) {
                *x = v;
            }
        }
        let [mut a, mut b, mut c, mut d, mut e, mut f, mut g, mut hh] = h;
        for (ki, wi) in K.iter().zip(w.iter()) {
            let s1 = e.rotate_right(6) ^ e.rotate_right(11) ^ e.rotate_right(25);
            let ch = (e & f) ^ (!e & g);
            let t1 = hh.wrapping_add(s1).wrapping_add(ch).wrapping_add(*ki).wrapping_add(*wi);
            let s0 = a.rotate_right(2) ^ a.rotate_right(13) ^ a.rotate_right(22);
            let maj = (a & b) ^ (a & c) ^ (b & c);
            let t2 = s0.wrapping_add(maj);
            hh = g;
            g = f;
            f = e;
            e = d.wrapping_add(t1);
            d = c;
            c = b;
            b = a;
            a = t1.wrapping_add(t2);
        }
        for (x, y) in h.iter_mut().zip([a, b, c, d, e, f, g, hh]) {
            *x = x.wrapping_add(y);
        }
    }
    h.iter().map(|x| format!("{x:08x}")).collect()
}

#[cfg(test)]
mod tests {
    #[test]
    fn sha256_of_abc_is_the_standard_vector() {
        assert_eq!(super::sha256_hex(b"abc"), "ba7816bf8f01cfea414140de5dae2223b00361a396177a9cb410ff61f20015ad");
        assert_eq!(super::sha256_hex(b""), "e3b0c44298fc1c149afbf4c8996fb92427ae41e4649b934ca495991b7852b855");
    }
}
