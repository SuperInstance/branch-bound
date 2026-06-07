//! Branch and bound node representation.

/// Status of a node in the search tree.
#[derive(Clone, Debug, PartialEq)]
pub enum NodeStatus {
    /// Not yet explored.
    Pending,
    /// Currently being explored.
    Active,
    /// Fully explored.
    Explored,
    /// Pruned (not worth exploring).
    Pruned,
    /// Infeasible (violates constraints).
    Infeasible,
}

/// A node in the branch and bound search tree.
#[derive(Clone, Debug)]
pub struct BNode {
    /// Level/depth in the search tree.
    pub level: usize,
    /// Partial solution (decisions made so far).
    pub partial: Vec<Option<i64>>,
    /// Lower bound on objective value for this subtree.
    pub bound: f64,
    /// Status of this node.
    pub status: NodeStatus,
}

impl BNode {
    pub fn new(level: usize, partial: Vec<Option<i64>>, bound: f64) -> Self {
        Self { level, partial, bound, status: NodeStatus::Pending }
    }

    /// Check if this node represents a complete solution.
    pub fn is_complete(&self, total_vars: usize) -> bool {
        self.partial.len() == total_vars && self.partial.iter().all(|v| v.is_some())
    }

    /// Branch: create two child nodes by fixing the next variable.
    pub fn branch(&self, var_idx: usize, _var_count: usize) -> (BNode, BNode) {
        let mut left_partial = self.partial.clone();
        while left_partial.len() <= var_idx {
            left_partial.push(None);
        }
        left_partial[var_idx] = Some(0);
        let left = BNode::new(self.level + 1, left_partial, 0.0);

        let mut right_partial = self.partial.clone();
        while right_partial.len() <= var_idx {
            right_partial.push(None);
        }
        right_partial[var_idx] = Some(1);
        let right = BNode::new(self.level + 1, right_partial, 0.0);

        (left, right)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn node_creation() {
        let node = BNode::new(0, vec![None, None], 10.0);
        assert_eq!(node.level, 0);
        assert_eq!(node.status, NodeStatus::Pending);
    }

    #[test]
    fn is_complete_true() {
        let node = BNode::new(2, vec![Some(1), Some(0)], 5.0);
        assert!(node.is_complete(2));
    }

    #[test]
    fn is_complete_false() {
        let node = BNode::new(1, vec![Some(1), None], 5.0);
        assert!(!node.is_complete(2));
    }

    #[test]
    fn branch_creates_children() {
        let node = BNode::new(0, vec![], 10.0);
        let (left, right) = node.branch(0, 2);
        assert_eq!(left.partial[0], Some(0));
        assert_eq!(right.partial[0], Some(1));
        assert_eq!(left.level, 1);
        assert_eq!(right.level, 1);
    }
}
