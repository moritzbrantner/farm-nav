use farm_nav::{NavDomain, PathRequest, astar};

#[derive(Clone, Copy, Debug, Eq, Hash, Ord, PartialEq, PartialOrd)]
struct Node(i32, i32);

#[derive(Clone, Copy, Debug, Eq, Hash, Ord, PartialEq, PartialOrd)]
enum Profile {
    Normal,
    BlockerBreaker,
}

struct ProfileGrid;

impl NavDomain for ProfileGrid {
    type Node = Node;
    type Profile = Profile;

    fn neighbors(
        &self,
        node: Self::Node,
        profile: Self::Profile,
        out: &mut Vec<(Self::Node, f32)>,
    ) {
        out.clear();
        for (dx, dy) in [(1, 0), (-1, 0), (0, 1), (0, -1)] {
            let next = Node(node.0 + dx, node.1 + dy);
            if !(0..=2).contains(&next.0) || !(0..=1).contains(&next.1) {
                continue;
            }
            if next == Node(1, 0) {
                match profile {
                    Profile::Normal => continue,
                    Profile::BlockerBreaker => {
                        out.push((next, 1.5));
                        continue;
                    }
                }
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
fn different_profiles_produce_different_routes() {
    let normal = astar(
        &ProfileGrid,
        PathRequest {
            start: Node(0, 0),
            goal: Node(2, 0),
            profile: Profile::Normal,
            allow_partial: false,
        },
    );
    let blocker_breaker = astar(
        &ProfileGrid,
        PathRequest {
            start: Node(0, 0),
            goal: Node(2, 0),
            profile: Profile::BlockerBreaker,
            allow_partial: false,
        },
    );

    assert!(normal.found);
    assert_eq!(
        normal.nodes,
        vec![Node(0, 0), Node(0, 1), Node(1, 1), Node(2, 1), Node(2, 0)]
    );
    assert!(blocker_breaker.found);
    assert_eq!(
        blocker_breaker.nodes,
        vec![Node(0, 0), Node(1, 0), Node(2, 0)]
    );
    assert!(blocker_breaker.total_cost < normal.total_cost);
}
