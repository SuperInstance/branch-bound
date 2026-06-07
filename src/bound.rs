//! Bounding functions for branch and bound.

/// Trait for bounding functions.
pub trait BoundingFunction {
    /// Compute a lower bound for the subtree rooted at the given partial solution.
    fn lower_bound(&self, partial: &[Option<i64>], values: &[f64], weights: &[f64], capacity: f64) -> f64;
}

/// Fractional relaxation bound (for knapsack-type problems).
pub struct FractionalBound;

impl BoundingFunction for FractionalBound {
    fn lower_bound(&self, partial: &[Option<i64>], values: &[f64], weights: &[f64], capacity: f64) -> f64 {
        let n = values.len();
        let mut remaining_capacity = capacity;
        let mut bound = 0.0;

        // Account for fixed variables
        for i in 0..partial.len().min(n) {
            if let Some(v) = partial[i] {
                if v == 1 {
                    bound += values[i];
                    remaining_capacity -= weights[i];
                }
            }
        }

        if remaining_capacity < 0.0 {
            return f64::NEG_INFINITY; // Infeasible
        }

        // Fractional relaxation: greedily fill remaining capacity
        let mut ratios: Vec<(usize, f64)> = (partial.len()..n)
            .map(|i| (i, values[i] / weights[i]))
            .collect();
        ratios.sort_by(|a, b| b.1.partial_cmp(&a.1).unwrap());

        for (idx, _) in ratios {
            if remaining_capacity <= 0.0 {
                break;
            }
            let take = remaining_capacity.min(weights[idx]);
            bound += values[idx] * (take / weights[idx]);
            remaining_capacity -= take;
        }

        bound
    }
}

/// Simple LP relaxation bound using continuous variables.
pub struct LPRelaxationBound;

impl BoundingFunction for LPRelaxationBound {
    fn lower_bound(&self, partial: &[Option<i64>], values: &[f64], weights: &[f64], capacity: f64) -> f64 {
        // Same as fractional for knapsack problems
        FractionalBound.lower_bound(partial, values, weights, capacity)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn fractional_bound_empty() {
        let bound = FractionalBound.lower_bound(
            &[], &[10.0, 20.0, 30.0], &[5.0, 10.0, 15.0], 20.0,
        );
        assert!(bound > 0.0);
    }

    #[test]
    fn fractional_bound_infeasible() {
        let bound = FractionalBound.lower_bound(
            &[Some(1)], &[10.0], &[100.0], 10.0,
        );
        assert_eq!(bound, f64::NEG_INFINITY);
    }

    #[test]
    fn fractional_bound_with_fixed() {
        let bound = FractionalBound.lower_bound(
            &[Some(1)], &[10.0, 20.0], &[5.0, 10.0], 15.0,
        );
        // Item 0 taken: value=10, weight=5, capacity left=10
        // Item 1: value/weight=2, take all: value=20, weight=10
        // Total: 30
        assert!((bound - 30.0).abs() < 1e-10);
    }

    #[test]
    fn lp_relaxation_matches_fractional() {
        let partial = &[Some(0)];
        let values = &[10.0, 20.0];
        let weights = &[5.0, 10.0];
        let capacity = 20.0;
        let fb = FractionalBound.lower_bound(partial, values, weights, capacity);
        let lp = LPRelaxationBound.lower_bound(partial, values, weights, capacity);
        assert!((fb - lp).abs() < 1e-10);
    }
}
