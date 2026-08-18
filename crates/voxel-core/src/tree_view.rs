//! The read-only view meshers consume, decoupling them from concrete
//! storage. Anything that can present itself as an octree of binary voxels
//! (today [`crate::octree::Octree`]; later e.g. a GridMap presenting a
//! virtual fully-subdivided tree) can be meshed.

use crate::octree::Node;
use crate::voxel::Voxel;

/// A handle to one octree node. Handles are small and `Copy`; geometry
/// (position, size) is *not* part of the handle — traversals track it
/// themselves from the root's bounds, which is what lets one node type
/// serve trees rooted at different places (chunks, absent neighbours).
///
/// Child octant indexing follows [`crate::octree::Node`]: bit 0 = +x,
/// bit 1 = +y, bit 2 = +z.
pub trait TreeNode: Copy {
    /// `Some(children)` for a branch, `None` for a leaf.
    fn children(self) -> Option<[Self; 8]>;

    /// The voxel stored at a leaf. Only meaningful when [`Self::children`]
    /// returned `None`; may panic otherwise.
    fn voxel(self) -> Voxel;

    /// Convenience: leaf-ness without pattern matching.
    fn is_leaf(self) -> bool {
        self.children().is_none()
    }
}

impl<'a> TreeNode for &'a Node {
    fn children(self) -> Option<[&'a Node; 8]> {
        match self {
            Node::Branch(children) => Some(children.each_ref()),
            Node::Leaf(_) => None,
        }
    }

    fn voxel(self) -> Voxel {
        match self {
            Node::Leaf(v) => *v,
            Node::Branch(_) => panic!("TreeNode::voxel called on a branch"),
        }
    }
}

/// The canonical all-inactive tree, standing in for chunks that don't
/// exist. Passing this as a neighbour is what closes meshes at the edge of
/// the world without any special-casing.
pub static EMPTY_NODE: Node = Node::Leaf(Voxel::INACTIVE);

#[cfg(test)]
mod tests {
    use super::*;
    use crate::octree::Octree;
    use glam::IVec3;

    #[test]
    fn tree_node_walks_match_octree_get() {
        let mut tree = Octree::uniform(2, Voxel::INACTIVE);
        tree.set(IVec3::new(3, 2, 1), Voxel::ACTIVE);

        // Walk to (3, 2, 1) by octant bits: at half=2 octant x=1,y=1,z=0 →
        // 0b011; at half=1 remaining (1, 0, 1) → octant 0b101.
        let root: &Node = tree.root();
        let children = root.children().expect("root must be split");
        let child = children[0b011];
        let grand = child.children().expect("child must be split")[0b101];
        assert!(grand.is_leaf());
        assert_eq!(grand.voxel(), Voxel::ACTIVE);

        let empty: &Node = &EMPTY_NODE;
        assert!(empty.is_leaf());
        assert_eq!(empty.voxel(), Voxel::INACTIVE);
    }
}
