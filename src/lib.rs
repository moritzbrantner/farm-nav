use std::cmp::Ordering;
use std::collections::{BTreeMap, BTreeSet, BinaryHeap, VecDeque};
use std::hash::Hash;

const EPSILON: f32 = 0.000_01;

pub trait NavDomain {
    type Node: Copy + Eq + Ord + Hash;
    type Profile: Copy + Eq + Ord + Hash;

    fn neighbors(&self, node: Self::Node, profile: Self::Profile, out: &mut Vec<(Self::Node, f32)>);

    fn heuristic(&self, from: Self::Node, goal: Self::Node, profile: Self::Profile) -> f32;

    fn version(&self) -> u64;
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub struct PathRequest<N, P> {
    pub start: N,
    pub goal: N,
    pub profile: P,
    pub allow_partial: bool,
}

#[derive(Clone, Debug, PartialEq)]
pub struct PathResult<N> {
    pub found: bool,
    pub nodes: Vec<N>,
    pub step_count: usize,
    pub total_cost: f32,
    pub next_step: Option<N>,
    pub visited_nodes: usize,
}

impl<N> PathResult<N> {
    pub fn not_found() -> Self {
        Self {
            found: false,
            nodes: Vec::new(),
            step_count: 0,
            total_cost: f32::INFINITY,
            next_step: None,
            visited_nodes: 0,
        }
    }
}

#[derive(Clone, Copy, Debug, Eq, Ord, PartialEq, PartialOrd)]
pub struct NavCacheKey<N, P> {
    pub version: u64,
    pub start: N,
    pub goal: N,
    pub profile: P,
}

#[derive(Clone, Debug)]
pub struct PathCache<N, P> {
    capacity: usize,
    entries: BTreeMap<NavCacheKey<N, P>, PathResult<N>>,
    order: VecDeque<NavCacheKey<N, P>>,
}

impl<N, P> PathCache<N, P>
where
    N: Clone + Ord,
    P: Clone + Ord,
{
    pub fn new(capacity: usize) -> Self {
        Self {
            capacity: capacity.max(1),
            entries: BTreeMap::new(),
            order: VecDeque::new(),
        }
    }

    pub fn len(&self) -> usize {
        self.entries.len()
    }

    pub fn get_cloned(&mut self, key: &NavCacheKey<N, P>) -> Option<PathResult<N>> {
        let value = self.entries.get(key).cloned()?;
        self.touch(key);
        Some(value)
    }

    pub fn insert(&mut self, key: NavCacheKey<N, P>, value: PathResult<N>) {
        self.entries.insert(key.clone(), value);
        self.touch(&key);
        while self.entries.len() > self.capacity {
            if let Some(oldest) = self.order.pop_front() {
                self.entries.remove(&oldest);
            }
        }
    }

    pub fn get_or_insert_with<F>(&mut self, key: NavCacheKey<N, P>, compute: F) -> PathResult<N>
    where
        F: FnOnce() -> PathResult<N>,
    {
        if let Some(value) = self.get_cloned(&key) {
            return value;
        }

        let value = compute();
        self.insert(key, value.clone());
        value
    }

    fn touch(&mut self, key: &NavCacheKey<N, P>) {
        self.order.retain(|existing| existing != key);
        self.order.push_back(key.clone());
    }
}

#[derive(Clone, Copy, Debug, PartialEq)]
struct QueueNode<N> {
    node: N,
    priority: f32,
}

impl<N: Eq> Eq for QueueNode<N> {}

impl<N: Ord> Ord for QueueNode<N> {
    fn cmp(&self, other: &Self) -> Ordering {
        other
            .priority
            .total_cmp(&self.priority)
            .then_with(|| self.node.cmp(&other.node))
    }
}

impl<N: Ord> PartialOrd for QueueNode<N> {
    fn partial_cmp(&self, other: &Self) -> Option<Ordering> {
        Some(self.cmp(other))
    }
}

#[derive(Clone, Copy, Debug)]
struct PartialCandidate<N> {
    node: N,
    heuristic: f32,
    cost: f32,
}

pub fn astar<D>(domain: &D, request: PathRequest<D::Node, D::Profile>) -> PathResult<D::Node>
where
    D: NavDomain,
{
    if request.start == request.goal {
        return PathResult {
            found: true,
            nodes: vec![request.start],
            step_count: 0,
            total_cost: 0.0,
            next_step: None,
            visited_nodes: 1,
        };
    }

    let mut open = BinaryHeap::new();
    let mut came_from = BTreeMap::<D::Node, D::Node>::new();
    let mut costs = BTreeMap::from([(request.start, 0.0_f32)]);
    let mut neighbors = Vec::new();
    let mut visited_nodes = 0_usize;
    let mut best_partial = PartialCandidate {
        node: request.start,
        heuristic: domain.heuristic(request.start, request.goal, request.profile),
        cost: 0.0,
    };

    open.push(QueueNode {
        node: request.start,
        priority: best_partial.heuristic,
    });

    while let Some(current) = open.pop() {
        let current_cost = costs.get(&current.node).copied().unwrap_or(f32::INFINITY);
        let expected_priority =
            current_cost + domain.heuristic(current.node, request.goal, request.profile);
        if current.priority > expected_priority + EPSILON {
            continue;
        }
        visited_nodes += 1;

        if current.node == request.goal {
            return build_path_result(
                &came_from,
                request.start,
                request.goal,
                current_cost,
                true,
                visited_nodes,
            );
        }

        neighbors.clear();
        domain.neighbors(current.node, request.profile, &mut neighbors);
        for &(neighbor, edge_cost) in &neighbors {
            if !edge_cost.is_finite() || edge_cost < 0.0 {
                continue;
            }

            let next_cost = current_cost + edge_cost;
            if next_cost + EPSILON < costs.get(&neighbor).copied().unwrap_or(f32::INFINITY) {
                costs.insert(neighbor, next_cost);
                came_from.insert(neighbor, current.node);

                let heuristic = domain.heuristic(neighbor, request.goal, request.profile);
                if heuristic + EPSILON < best_partial.heuristic
                    || ((heuristic - best_partial.heuristic).abs() <= EPSILON
                        && next_cost + EPSILON < best_partial.cost)
                {
                    best_partial = PartialCandidate {
                        node: neighbor,
                        heuristic,
                        cost: next_cost,
                    };
                }

                open.push(QueueNode {
                    node: neighbor,
                    priority: next_cost + heuristic,
                });
            }
        }
    }

    if request.allow_partial && best_partial.node != request.start {
        return build_path_result(
            &came_from,
            request.start,
            best_partial.node,
            best_partial.cost,
            false,
            visited_nodes,
        );
    }

    let mut result = PathResult::not_found();
    result.visited_nodes = visited_nodes;
    result
}

pub fn dijkstra_to_any<D>(
    domain: &D,
    start: D::Node,
    goals: &BTreeSet<D::Node>,
    profile: D::Profile,
) -> PathResult<D::Node>
where
    D: NavDomain,
{
    if goals.is_empty() {
        return PathResult::not_found();
    }
    if goals.contains(&start) {
        return PathResult {
            found: true,
            nodes: vec![start],
            step_count: 0,
            total_cost: 0.0,
            next_step: None,
            visited_nodes: 1,
        };
    }

    let mut open = BinaryHeap::new();
    let mut came_from = BTreeMap::<D::Node, D::Node>::new();
    let mut costs = BTreeMap::from([(start, 0.0_f32)]);
    let mut neighbors = Vec::new();
    let mut visited_nodes = 0_usize;

    open.push(QueueNode {
        node: start,
        priority: 0.0,
    });

    while let Some(current) = open.pop() {
        let current_cost = costs.get(&current.node).copied().unwrap_or(f32::INFINITY);
        if current.priority > current_cost + EPSILON {
            continue;
        }
        visited_nodes += 1;

        if goals.contains(&current.node) {
            return build_path_result(
                &came_from,
                start,
                current.node,
                current_cost,
                true,
                visited_nodes,
            );
        }

        neighbors.clear();
        domain.neighbors(current.node, profile, &mut neighbors);
        for &(neighbor, edge_cost) in &neighbors {
            if !edge_cost.is_finite() || edge_cost < 0.0 {
                continue;
            }

            let next_cost = current_cost + edge_cost;
            if next_cost + EPSILON < costs.get(&neighbor).copied().unwrap_or(f32::INFINITY) {
                costs.insert(neighbor, next_cost);
                came_from.insert(neighbor, current.node);
                open.push(QueueNode {
                    node: neighbor,
                    priority: next_cost,
                });
            }
        }
    }

    let mut result = PathResult::not_found();
    result.visited_nodes = visited_nodes;
    result
}

pub fn reachable_nodes<D>(
    domain: &D,
    start: D::Node,
    max_cost: Option<f32>,
    profile: D::Profile,
) -> BTreeMap<D::Node, f32>
where
    D: NavDomain,
{
    let mut open = BinaryHeap::new();
    let mut costs = BTreeMap::from([(start, 0.0_f32)]);
    let mut neighbors = Vec::new();
    let limit = max_cost.unwrap_or(f32::INFINITY);

    open.push(QueueNode {
        node: start,
        priority: 0.0,
    });

    while let Some(current) = open.pop() {
        let current_cost = costs.get(&current.node).copied().unwrap_or(f32::INFINITY);
        if current.priority > current_cost + EPSILON || current_cost > limit + EPSILON {
            continue;
        }

        neighbors.clear();
        domain.neighbors(current.node, profile, &mut neighbors);
        for &(neighbor, edge_cost) in &neighbors {
            if !edge_cost.is_finite() || edge_cost < 0.0 {
                continue;
            }

            let next_cost = current_cost + edge_cost;
            if next_cost > limit + EPSILON {
                continue;
            }
            if next_cost + EPSILON < costs.get(&neighbor).copied().unwrap_or(f32::INFINITY) {
                costs.insert(neighbor, next_cost);
                open.push(QueueNode {
                    node: neighbor,
                    priority: next_cost,
                });
            }
        }
    }

    costs
}

fn build_path_result<N>(
    came_from: &BTreeMap<N, N>,
    start: N,
    goal: N,
    total_cost: f32,
    found: bool,
    visited_nodes: usize,
) -> PathResult<N>
where
    N: Copy + Ord,
{
    let nodes = reconstruct_path(came_from, start, goal);
    PathResult {
        found,
        step_count: nodes.len().saturating_sub(1),
        next_step: nodes.get(1).copied(),
        nodes,
        total_cost,
        visited_nodes,
    }
}

fn reconstruct_path<N>(came_from: &BTreeMap<N, N>, start: N, goal: N) -> Vec<N>
where
    N: Copy + Ord,
{
    let mut path = vec![goal];
    let mut current = goal;
    while current != start {
        let Some(previous) = came_from.get(&current).copied() else {
            break;
        };
        current = previous;
        path.push(current);
    }
    path.reverse();
    path
}
