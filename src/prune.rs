//! Pruning strategies for branch and bound.

/// Trait for pruning decisions.
pub trait PruningRule {
    /// Returns true if the node should be pruned.
    fn should_prune(&self, bound: f64, best_known: f64, is_feasible: bool) -> bool;
}

/// Prune if bound is worse than best known solution.
pub struct BoundPruning;

impl PruningRule for BoundPruning {
    fn should_prune(&self, bound: f64, best_known: f64, _is_feasible: bool) -> bool {
        bound <= best_known
    }
}

/// Prune infeasible nodes and bound-exceeded nodes.
pub struct AggressivePruning {
    pub tolerance: f64,
}

impl PruningRule for AggressivePruning {
    fn should_prune(&self, bound: f64, best_known: f64, is_feasible: bool) -> bool {
        if !is_feasible {
            return true;
        }
        bound <= best_known + self.tolerance
    }
}

/// No pruning (explore everything).
pub struct NoPruning;

impl PruningRule for NoPruning {
    fn should_prune(&self, _bound: f64, _best_known: f64, _is_feasible: bool) -> bool {
        false
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn bound_pruning_worse_bound() {
        assert!(BoundPruning.should_prune(5.0, 10.0, true));
    }

    #[test]
    fn bound_pruning_better_bound() {
        assert!(!BoundPruning.should_prune(15.0, 10.0, true));
    }

    #[test]
    fn aggressive_prunes_infeasible() {
        let ap = AggressivePruning { tolerance: 0.0 };
        assert!(ap.should_prune(100.0, 10.0, false));
    }

    #[test]
    fn aggressive_with_tolerance() {
        let ap = AggressivePruning { tolerance: 1.0 };
        assert!(ap.should_prune(9.5, 10.0, true));
        assert!(!ap.should_prune(12.0, 10.0, true));
    }

    #[test]
    fn no_pruning_never_prunes() {
        assert!(!NoPruning.should_prune(f64::NEG_INFINITY, f64::MAX, false));
    }
}
