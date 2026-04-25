use farm_nav::{NavDomain, PathRequest, astar};

#[derive(Clone, Copy, Debug, Eq, Hash, Ord, PartialEq, PartialOrd)]
struct Node(i32, i32);

#[derive(Clone, Copy, Debug, Eq, Hash, Ord, PartialEq, PartialOrd)]
enum Profile {
    Worker,
}

struct WeightedGrid;

impl NavDomain for WeightedGrid {
    type Node = Node;
    type Profile = Profile;

    fn neighbors(&self, node: Self::Node, _: Self::Profile, out: &mut Vec<(Self::Node, f32)>) {
        out.clear();
        for (dx, dy) in [(1, 0), (-1, 0), (0, 1), (0, -1)] {
            let next = Node(node.0 + dx, node.1 + dy);
            if !(0..=2).contains(&next.0) || !(0..=2).contains(&next.1) {
                continue;
            }
            let cost = if next.1 == 0 { 0.5 } else { 2.0 };
            out.push((next, cost));
        }
    }

    fn heuristic(&self, from: Self::Node, goal: Self::Node, _: Self::Profile) -> f32 {
        ((from.0 - goal.0).abs() + (from.1 - goal.1).abs()) as f32 * 0.5
    }

    fn version(&self) -> u64 {
        1
    }
}

#[test]
fn weighted_road_path_beats_plain_terrain() {
    let result = astar(
        &WeightedGrid,
        PathRequest {
            start: Node(0, 1),
            goal: Node(2, 1),
            profile: Profile::Worker,
            allow_partial: false,
        },
    );

    assert!(result.found);
    assert_eq!(
        result.nodes,
        vec![Node(0, 1), Node(0, 0), Node(1, 0), Node(2, 0), Node(2, 1)]
    );
    assert!(result.total_cost < 4.0);
}
