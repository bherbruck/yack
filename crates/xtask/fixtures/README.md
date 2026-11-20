# xtask gate fixtures

`base/` is a tiny workspace that passes every `cargo xtask gate` check. Each `fail_<NN>_<what>/`
overlays files on it (same relative paths) to plant one violation of check NN (PLAN §6.6); the
tests in `src/gate/mod.rs` require that check NN fails and no other check fails unless listed in
`_expect.txt`. Each `pass_<NN>_<what>/` must pass every check (e.g. a temporary crate held at
its ratchet).

Naming: `dot-x` stands for `.x` and a trailing `.in` is dropped (so no fixture manifest or dotfile
acts on the real repository). Control files: `_delete.txt` (paths to remove), `_dirs.txt`
(forbidden directories that exist), `_tags.txt` (git tags), `_expect.txt` (extra failing checks).
