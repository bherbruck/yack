// yack: the SMLib corpus runner: one case through yack's public API, its result's measures printed.
//! `yack-corpus CASES.tsv ID`, run by `cargo xtask corpus smlib` once per case (one process each,
//! so a crash is that case's alone). It reads the case's inputs, which are STEP files made at
//! fetch time, with `yack::step::read`, runs the operation, and prints one line:
//!
//! ```text
//! ok <volume> <area> <faces> <edges> <valid> <ms>
//! refused <ms> <why>
//! ```
//!
//! Grading against SMLib's recorded results is the caller's (xtask); nothing here knows them.
//! CASES.tsv rows: `id op size sharp inputs edge-points face-points`, tab-separated; lists are
//! `;`-separated and points `x,y,z`.
use std::process::ExitCode;

use yack::prelude::*;

/// A runner-side problem (a bad case file), as text.
type Res<T> = std::result::Result<T, String>;

struct Case {
    op: String,
    size: f64,
    sharp: bool,
    inputs: Vec<String>,
    edges: Vec<Point3>,
    faces: Vec<Point3>,
}

fn main() -> ExitCode {
    let args: Vec<String> = std::env::args().skip(1).collect();
    let (Some(file), Some(id)) = (args.first(), args.get(1)) else {
        eprintln!("usage: yack-corpus CASES.tsv ID");
        return ExitCode::FAILURE;
    };
    let case = match load(file, id) {
        Ok(c) => c,
        Err(e) => {
            eprintln!("yack-corpus: {e}");
            return ExitCode::FAILURE;
        }
    };
    // wall-clock time of the case, for the report only (a tool, not kernel code)
    #[allow(clippy::disallowed_methods)]
    let t0 = std::time::Instant::now();
    let r = build(&case);
    let ms = t0.elapsed().as_secs_f64() * 1e3;
    match r {
        Ok(s) => {
            let valid = s.check().is_empty();
            println!("ok {} {} {} {} {valid} {ms:.1}", s.volume(), s.area(), s.faces().len(), s.edges().len());
        }
        Err(e) => println!("refused {ms:.1} {e}"),
    }
    ExitCode::SUCCESS
}

fn points(field: &str) -> Res<Vec<Point3>> {
    field
        .split(';')
        .filter(|s| !s.is_empty())
        .map(|p| {
            let c: Vec<f64> = p.split(',').map(|x| x.parse::<f64>().map_err(|e| format!("{p}: {e}"))).collect::<Res<_>>()?;
            match c.as_slice() {
                [x, y, z] => Ok(Point3::new(*x, *y, *z)),
                _ => Err(format!("{p}: not a point")),
            }
        })
        .collect()
}

fn load(file: &str, id: &str) -> Res<Case> {
    let text = std::fs::read_to_string(file).map_err(|e| format!("{file}: {e}"))?;
    let row = text.lines().find(|l| l.split('\t').next() == Some(id)).ok_or(format!("no case {id} in {file}"))?;
    let f: Vec<&str> = row.split('\t').collect();
    let [_, op, size, sharp, inputs, edges, faces, ..] = f.as_slice() else { return Err(format!("{id}: a row needs 7 fields")) };
    Ok(Case {
        op: (*op).to_string(),
        size: size.parse::<f64>().map_err(|e| format!("{id} size: {e}"))?,
        sharp: *sharp == "true",
        inputs: inputs.split(';').filter(|s| !s.is_empty()).map(str::to_string).collect(),
        edges: points(edges)?,
        faces: points(faces)?,
    })
}

fn read(path: &str) -> Result<Solid> {
    let text = std::fs::read_to_string(path).map_err(|e| Error::new(ErrorKind::NotFound, "yack-corpus", format!("{path}: {e}")))?;
    yack::step::read(&text)
}

fn build(c: &Case) -> Result<Solid> {
    let mut solids = c.inputs.iter().map(|p| read(p)).collect::<Result<Vec<_>>>()?.into_iter();
    let first = solids.next().ok_or_else(|| Error::new(ErrorKind::Invalid, "yack-corpus", "no inputs"))?;
    match c.op.as_str() {
        "union" => solids.try_fold(first, |acc, s| acc.union(&s)),
        "intersection" => solids.try_fold(first, |acc, s| acc.intersection(&s)),
        "difference" => solids.try_fold(first, |acc, s| acc.difference(&s)),
        "fillet" | "chamfer" => {
            let edges = c.edges.iter().map(|&p| first.edge_at(p)).collect::<Result<Vec<_>>>()?;
            if c.op == "fillet" { first.fillet(&edges, c.size) } else { first.chamfer(&edges, c.size) }
        }
        "shell" => {
            let open = c.faces.iter().map(|&p| first.face_at(p)).collect::<Result<Vec<_>>>()?;
            // SMLib's thickness: negative inward, positive outward; sharp joins or rounded ones
            let t = if c.size < 0.0 { -c.size } else { c.size };
            match (c.size > 0.0, c.sharp) {
                (false, true) => first.shell(&open, t),
                (false, false) => first.shell_rounded(&open, t),
                (true, true) => first.shell_outward(&open, t),
                (true, false) => first.shell_outward_rounded(&open, t),
            }
        }
        "offset" => first.offset(c.size),
        other => Err(Error::new(ErrorKind::Unsupported, "yack-corpus", format!("operation {other}"))),
    }
}
