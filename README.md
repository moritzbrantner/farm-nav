# farm-nav

Deterministic, dependency-free pathfinding for grid and graph games.

`farm_nav` searches any caller-supplied graph that implements `NavDomain` (neighbors, heuristic, and a domain version). It provides:

- `astar` with optional partial paths toward the closest reachable node,
- `dijkstra_to_any` for nearest-of-many goals,
- `reachable_nodes` for flood-fill reachability,
- movement profiles so one domain can serve several agent types,
- `PathCache`, a bounded cache keyed by domain version, endpoints and profile.

Ties are broken by stable node ordering, so identical inputs give identical paths.

## Usage

Pin a commit in `Cargo.toml`:

```toml
[dependencies]
farm_nav = { git = "https://github.com/moritzbrantner/farm-nav", rev = "<commit>" }
```

See `tests/` for complete grid examples.

## Development

```sh
cargo fmt --all -- --check
cargo clippy --all-targets -- -D warnings
cargo test
```

## Origin

Extracted from the private farm game engine (`crates/farm_nav`) with its history preserved.

## License

MIT
