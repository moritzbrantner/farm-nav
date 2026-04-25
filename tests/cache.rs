use farm_nav::{NavCacheKey, PathCache, PathResult};

#[derive(Clone, Copy, Debug, Eq, Ord, PartialEq, PartialOrd)]
struct Node(i32, i32);

#[derive(Clone, Copy, Debug, Eq, Ord, PartialEq, PartialOrd)]
enum Profile {
    Worker,
}

#[test]
fn cache_invalidates_when_version_changes() {
    let mut cache = PathCache::new(4);
    let original_key = NavCacheKey {
        version: 1,
        start: Node(0, 0),
        goal: Node(2, 0),
        profile: Profile::Worker,
    };
    let updated_key = NavCacheKey {
        version: 2,
        ..original_key
    };

    let original = PathResult {
        found: true,
        nodes: vec![Node(0, 0), Node(1, 0), Node(2, 0)],
        step_count: 2,
        total_cost: 2.0,
        next_step: Some(Node(1, 0)),
        visited_nodes: 3,
    };

    cache.insert(original_key, original.clone());

    assert_eq!(cache.get_cloned(&updated_key), None);
    assert_eq!(cache.get_cloned(&original_key), Some(original));
}
