# Agent guidance

`farm_nav` is a small, deterministic, domain-agnostic pathfinding crate (A* over a caller-supplied `NavDomain`, movement profiles, blockers and a versioned path cache). It has no dependencies and must stay free of game, ECS, rendering or engine concepts.

- Keep results deterministic: break ties by stable node ordering, never by hash iteration order.
- Keep the public API generic over `NavDomain`; domain-specific grids belong in consumers.
- Prefer small, focused modules; treat 500 lines per hand-written file as a warning threshold.
- Before a PR: `cargo fmt --all -- --check`, `cargo clippy --all-targets -- -D warnings`, `cargo test`.

Consumers (for example `moritzbrantner/settlement` and the farm game engine) pin this repo by git commit `rev`.
