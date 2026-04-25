use farm_nav::{NavDomain, PathRequest, astar};

#[derive(Clone, Copy, Debug, Eq, Hash, Ord, PartialEq, PartialOrd)]
struct Node(i32, i32);

#[derive(Clone, Copy, Debug, Eq, Hash, Ord, PartialEq, PartialOrd)]
enum Profile {
    Worker,
}

struct BlockedGrid;

impl NavDomain for BlockedGrid {
    type Node = Node;
    type Profile = Profile;

    fn neighbors(&self, node: Self::Node, _: Self::Profile, out: &mut Vec<(Self::Node, f32)>) {
        out.clear();
        for (dx, dy) in [(1, 0), (-1, 0), (0, 1), (0, -1)] {
            let next = Node(node.0 + dx, node.1 + dy);
            if !(0..=2).contains(&next.0) || next.1 != 1 || next == Node(1, 1) {
                continue;
            }
            out.push((next, 1.0));
        }
    }

    fn heuristic(&self, from: Self::Node, goal: Self::Node, _: Self::Profile) -> f32 {
        ((from.0 - goal.0).abs() + (from.1 - goal.1).abs()) as f32
    }

    fn version(&self) -> u64 {
        1
    }
}

#[test]
fn hard_blockers_prevent_routes() {
    let result = astar(
        &BlockedGrid,
        PathRequest {
            start: Node(0, 1),
            goal: Node(2, 1),
            profile: Profile::Worker,
            allow_partial: false,
        },
    );

    assert!(!result.found);
    assert!(result.nodes.is_empty());
}
