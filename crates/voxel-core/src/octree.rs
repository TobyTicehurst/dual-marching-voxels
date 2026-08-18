//! Sparse octree over a power-of-two cube of voxels.
//!
//! A leaf at size `s` is one voxel of size `s` (the multi-scale data model:
//! voxel sizes are always powers of two). Two invariants hold everywhere:
//!
//! - **Merged**: no branch has 8 leaf children carrying the same voxel value
//!   (such a branch is always collapsed to a leaf on construction/mutation).
//! - **Depth-bounded**: branches only exist for node sizes ≥ 2, so a
//!   descent always terminates at a leaf by the time size reaches 1.

use glam::IVec3;

use crate::voxel::Voxel;

/// One octree node. Child octant index encoding: bit 0 = +x half,
/// bit 1 = +y half, bit 2 = +z half. Everything that walks trees (including
/// the mesher's traversal tables in other crates) relies on this encoding.
#[derive(Debug, Clone, PartialEq, Eq)]
pub enum Node {
    Leaf(Voxel),
    Branch(Box<[Node; 8]>),
}

/// The offset (in units of the child size) of a child octant within its
/// parent, per the bit encoding documented on [`Node`].
pub const fn octant_offset(octant: usize) -> IVec3 {
    IVec3::new(
        (octant & 1) as i32,
        ((octant >> 1) & 1) as i32,
        ((octant >> 2) & 1) as i32,
    )
}

/// Which octant of a node with child size `half` the local point `p` falls
/// in. `p` must lie in `[0, 2 * half)³`.
const fn octant_of(p: IVec3, half: i32) -> usize {
    (p.x >= half) as usize | (((p.y >= half) as usize) << 1) | (((p.z >= half) as usize) << 2)
}

impl Node {
    /// `Some(voxel)` if this node is a leaf.
    pub fn as_leaf(&self) -> Option<Voxel> {
        match self {
            Node::Leaf(v) => Some(*v),
            Node::Branch(_) => None,
        }
    }

    fn collapse_if_uniform(&mut self) {
        if let Node::Branch(children) = self
            && let Some(v) = children[0].as_leaf()
            && children.iter().all(|c| c.as_leaf() == Some(v))
        {
            *self = Node::Leaf(v);
        }
    }
}

/// An octree covering the cube `[0, side)³` where `side = 1 << depth`.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct Octree {
    root: Node,
    depth: u32,
}

impl Octree {
    pub fn uniform(depth: u32, voxel: Voxel) -> Self {
        Octree {
            root: Node::Leaf(voxel),
            depth,
        }
    }

    /// Builds by sampling `f` at every finest-scale position. Uniform
    /// regions merge into single larger leaves, so sampling a function that
    /// is constant over aligned power-of-two blocks yields genuinely
    /// coarse voxels.
    pub fn from_fn(depth: u32, f: &mut impl FnMut(IVec3) -> Voxel) -> Self {
        Octree {
            root: build_from_fn(IVec3::ZERO, 1 << depth, f),
            depth,
        }
    }

    pub fn depth(&self) -> u32 {
        self.depth
    }

    pub fn side(&self) -> u32 {
        1 << self.depth
    }

    pub fn root(&self) -> &Node {
        &self.root
    }

    /// `Some(voxel)` if the whole tree is one uniform voxel.
    pub fn as_uniform(&self) -> Option<Voxel> {
        self.root.as_leaf()
    }

    /// The finest-scale voxel value covering position `p`.
    ///
    /// Panics if `p` is outside `[0, side)³` — callers wanting "outside
    /// reads as inactive" semantics get them from [`crate::world::World`],
    /// not here.
    pub fn get(&self, p: IVec3) -> Voxel {
        assert!(self.in_bounds(p), "Octree::get out of bounds: {p}");
        let mut node = &self.root;
        let mut half = (self.side() / 2) as i32;
        let mut p = p;
        loop {
            match node {
                Node::Leaf(v) => return *v,
                Node::Branch(children) => {
                    let o = octant_of(p, half);
                    p -= octant_offset(o) * half;
                    node = &children[o];
                    half /= 2;
                }
            }
        }
    }

    /// Sets one finest-scale voxel.
    pub fn set(&mut self, p: IVec3, voxel: Voxel) {
        self.set_region(p, 1, voxel);
    }

    /// Sets one voxel of size `size` (a power of two) with min corner `min`.
    ///
    /// Panics unless `size` is a power of two, `min` is aligned to `size`,
    /// and the region lies inside the tree. Alignment is what guarantees a
    /// region never straddles an octant boundary, so it always lands on a
    /// single node.
    pub fn set_region(&mut self, min: IVec3, size: u32, voxel: Voxel) {
        assert!(
            size.is_power_of_two() && size <= self.side(),
            "set_region: bad size {size} (side {})",
            self.side()
        );
        let s = size as i32;
        assert!(
            min % s == IVec3::ZERO,
            "set_region: min {min} not aligned to size {size}"
        );
        assert!(
            self.in_bounds(min) && self.in_bounds(min + s - 1),
            "set_region: region {min}+{size} out of bounds (side {})",
            self.side()
        );
        let side = self.side();
        set_region_node(&mut self.root, side, min, size, voxel);
    }

    /// Calls `f(min, size, voxel)` for every leaf, i.e. every stored voxel
    /// at its own scale. Leaves partition the cube exactly.
    pub fn for_each_leaf(&self, f: &mut impl FnMut(IVec3, u32, Voxel)) {
        for_each_leaf_node(&self.root, IVec3::ZERO, self.side(), f);
    }

    /// Number of leaves — a measure of how well merging is compressing.
    pub fn leaf_count(&self) -> usize {
        let mut n = 0;
        self.for_each_leaf(&mut |_, _, _| n += 1);
        n
    }

    fn in_bounds(&self, p: IVec3) -> bool {
        let side = self.side() as i32;
        p.cmpge(IVec3::ZERO).all() && p.cmplt(IVec3::splat(side)).all()
    }
}

fn build_from_fn(min: IVec3, size: u32, f: &mut impl FnMut(IVec3) -> Voxel) -> Node {
    if size == 1 {
        return Node::Leaf(f(min));
    }
    let half = (size / 2) as i32;
    let children: [Node; 8] =
        std::array::from_fn(|o| build_from_fn(min + octant_offset(o) * half, size / 2, f));
    let mut node = Node::Branch(Box::new(children));
    node.collapse_if_uniform();
    node
}

fn set_region_node(node: &mut Node, node_size: u32, min: IVec3, size: u32, voxel: Voxel) {
    if size == node_size {
        // Alignment means min == 0 here: the region is exactly this node.
        *node = Node::Leaf(voxel);
        return;
    }
    if let Node::Leaf(v) = node {
        if *v == voxel {
            return;
        }
        *node = Node::Branch(Box::new(std::array::from_fn(|_| Node::Leaf(*v))));
    }
    let Node::Branch(children) = node else {
        unreachable!()
    };
    let half = (node_size / 2) as i32;
    let o = octant_of(min, half);
    set_region_node(
        &mut children[o],
        node_size / 2,
        min - octant_offset(o) * half,
        size,
        voxel,
    );
    node.collapse_if_uniform();
}

fn for_each_leaf_node(node: &Node, min: IVec3, size: u32, f: &mut impl FnMut(IVec3, u32, Voxel)) {
    match node {
        Node::Leaf(v) => f(min, size, *v),
        Node::Branch(children) => {
            let half = (size / 2) as i32;
            for (o, child) in children.iter().enumerate() {
                for_each_leaf_node(child, min + octant_offset(o) * half, size / 2, f);
            }
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    const A: Voxel = Voxel::ACTIVE;
    const I: Voxel = Voxel::INACTIVE;

    /// Tiny deterministic PRNG so tests need no dependencies.
    struct Lcg(u64);
    impl Lcg {
        fn next(&mut self, bound: u32) -> u32 {
            self.0 = self
                .0
                .wrapping_mul(6364136223846793005)
                .wrapping_add(1442695040888963407);
            ((self.0 >> 33) as u32) % bound
        }
    }

    /// Checks the "merged" invariant: no branch may have 8 identical leaf
    /// children.
    fn assert_fully_merged(node: &Node) {
        if let Node::Branch(children) = node {
            let first = children[0].as_leaf();
            assert!(
                !(first.is_some() && children.iter().all(|c| c.as_leaf() == first)),
                "branch with 8 identical leaves should have been collapsed"
            );
            for c in children.iter() {
                assert_fully_merged(c);
            }
        }
    }

    #[test]
    fn get_matches_dense_reference_under_random_writes() {
        let depth = 3;
        let side = 1i32 << depth;
        let mut tree = Octree::uniform(depth, I);
        let mut dense = vec![I; (side * side * side) as usize];
        let idx = |p: IVec3| (p.x + side * (p.y + side * p.z)) as usize;

        let mut rng = Lcg(42);
        for _ in 0..300 {
            let size = 1u32 << rng.next(3); // 1, 2 or 4
            let s = size as i32;
            let cells = (side as u32) / size;
            let min = IVec3::new(
                rng.next(cells) as i32 * s,
                rng.next(cells) as i32 * s,
                rng.next(cells) as i32 * s,
            );
            let v = Voxel::new(rng.next(2) == 0);
            tree.set_region(min, size, v);
            for x in 0..s {
                for y in 0..s {
                    for z in 0..s {
                        dense[idx(min + IVec3::new(x, y, z))] = v;
                    }
                }
            }
        }

        for x in 0..side {
            for y in 0..side {
                for z in 0..side {
                    let p = IVec3::new(x, y, z);
                    assert_eq!(tree.get(p), dense[idx(p)], "mismatch at {p}");
                }
            }
        }
        assert_fully_merged(tree.root());
    }

    #[test]
    fn writing_the_same_value_everywhere_collapses_to_one_leaf() {
        let mut tree = Octree::uniform(2, I);
        for x in 0..4 {
            for y in 0..4 {
                for z in 0..4 {
                    tree.set(IVec3::new(x, y, z), A);
                }
            }
        }
        assert_eq!(tree.as_uniform(), Some(A));
        assert_eq!(tree.leaf_count(), 1);
    }

    #[test]
    fn mixed_scale_leaves_partition_the_cube() {
        let mut tree = Octree::uniform(3, I);
        tree.set_region(IVec3::ZERO, 4, A); // one 4³ voxel
        tree.set_region(IVec3::new(4, 0, 0), 2, A); // one 2³ voxel
        tree.set(IVec3::new(7, 7, 7), A); // one 1³ voxel

        let mut covered = vec![false; 512];
        let mut sizes_seen = Vec::new();
        tree.for_each_leaf(&mut |min, size, v| {
            sizes_seen.push(size);
            let s = size as i32;
            for x in 0..s {
                for y in 0..s {
                    for z in 0..s {
                        let p = min + IVec3::new(x, y, z);
                        let i = (p.x + 8 * (p.y + 8 * p.z)) as usize;
                        assert!(!covered[i], "leaves overlap at {p}");
                        covered[i] = true;
                        assert_eq!(tree.get(p), v);
                    }
                }
            }
        });
        assert!(covered.iter().all(|&c| c), "leaves must cover the cube");
        assert!(sizes_seen.contains(&4) && sizes_seen.contains(&2) && sizes_seen.contains(&1));
    }

    #[test]
    fn from_fn_merges_uniform_regions() {
        // Active only in the -x half: the two halves are uniform at size 4,
        // so the tree should be exactly 8 leaves (one per octant), not 512.
        let tree = Octree::from_fn(3, &mut |p| Voxel::new(p.x < 4));
        assert_eq!(tree.leaf_count(), 8);
        assert_fully_merged(tree.root());
        assert_eq!(tree.get(IVec3::new(0, 5, 5)), A);
        assert_eq!(tree.get(IVec3::new(4, 5, 5)), I);
    }

    #[test]
    fn from_fn_matches_the_function() {
        let mut f = |p: IVec3| Voxel::new((p.x + p.y * 3 + p.z * 7) % 5 < 2);
        let tree = Octree::from_fn(2, &mut f);
        for x in 0..4 {
            for y in 0..4 {
                for z in 0..4 {
                    let p = IVec3::new(x, y, z);
                    assert_eq!(tree.get(p), f(p), "mismatch at {p}");
                }
            }
        }
        assert_fully_merged(tree.root());
    }

    #[test]
    #[should_panic(expected = "not aligned")]
    fn misaligned_region_panics() {
        let mut tree = Octree::uniform(3, I);
        tree.set_region(IVec3::new(1, 0, 0), 2, A);
    }

    #[test]
    #[should_panic(expected = "out of bounds")]
    fn out_of_bounds_get_panics() {
        Octree::uniform(3, I).get(IVec3::new(8, 0, 0));
    }
}
