# scores

Committed ratchets (PLAN §6.4). Every file is sorted and timestamp-free, so a diff shows exactly
which case moved. `cargo xtask conformance --bless` and `cargo xtask bench --bless` update them in
the same commit as the change that earned it.

| File | Holds |
|---|---|
| `conformance/seed<N>.json` | per case: outcome, tier, time_ms |
| `bench.json` | per case: outcome, time, loc, strategy |
| `digests/seed0.txt` | per case: result digests (M2–M3 pure-move gate) |
| `waivers.toml` | the only allowed exceptions (case, reason, owner, expiry); never for `wrong` |
| `eps.txt`, `filesize.txt`, `cadrs-loc.txt` | counts that may only fall |
| `known-failing.txt` | a ledger that may only shrink |
