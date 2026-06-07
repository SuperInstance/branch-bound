//! Knapsack problem solver using branch and bound.

use crate::bound::BoundingFunction;
use crate::prune::PruningRule;
use crate::node::BNode;

/// 0/1 Knapsack problem.
pub struct Knapsack {
    pub values: Vec<f64>,
    pub weights: Vec<f64>,
    pub capacity: f64,
}

impl Knapsack {
    pub fn new(values: Vec<f64>, weights: Vec<f64>, capacity: f64) -> Self {
        assert_eq!(values.len(), weights.len());
        Self { values, weights, capacity }
    }

    /// Solve the knapsack problem using branch and bound.
    pub fn solve<B: BoundingFunction, P: PruningRule>(
        &self,
        bounding: &B,
        pruning: &P,
    ) -> KnapsackSolution {
        let n = self.values.len();
        let mut best_solution = vec![0i64; n];
        let mut best_value = 0.0_f64;

        let root = BNode::new(0, vec![], 0.0);
        let mut stack = vec![root];

        while let Some(node) = stack.pop() {
            let bound = bounding.lower_bound(&node.partial, &self.values, &self.weights, self.capacity);

            let is_feasible = self.is_feasible(&node.partial);
            if pruning.should_prune(bound, best_value, is_feasible) {
                continue;
            }

            let next_var = node.partial.len();
            if next_var >= n {
                // Complete solution
                let sol: Vec<i64> = node.partial.iter().map(|v| v.unwrap_or(0)).collect();
                let val = self.eval_solution(&sol);
                if val > best_value && self.is_feasible_complete(&sol) {
                    best_value = val;
                    best_solution = sol;
                }
                continue;
            }

            // Branch: take or skip
            let (mut take, mut skip) = node.branch(next_var, n);
            take.bound = bound;
            skip.bound = bound;
            // Push skip first, take second (DFS will explore take first)
            stack.push(skip);
            stack.push(take);
        }

        let total_weight: f64 = best_solution.iter().enumerate()
            .map(|(i, v)| self.weights[i] * *v as f64)
            .sum();

        KnapsackSolution {
            items: best_solution,
            total_value: best_value,
            total_weight,
        }
    }

    fn eval_solution(&self, sol: &[i64]) -> f64 {
        self.values.iter().zip(sol).map(|(v, x)| v * *x as f64).sum()
    }

    fn is_feasible(&self, partial: &[Option<i64>]) -> bool {
        let weight: f64 = partial.iter().enumerate()
            .filter(|(_, v)| **v == Some(1))
            .map(|(i, _)| self.weights[i])
            .sum();
        weight <= self.capacity
    }

    fn is_feasible_complete(&self, sol: &[i64]) -> bool {
        let weight: f64 = sol.iter().enumerate()
            .filter(|(_, v)| **v == 1)
            .map(|(i, _)| self.weights[i])
            .sum();
        weight <= self.capacity
    }
}

/// Solution to a knapsack problem.
#[derive(Debug, Clone)]
pub struct KnapsackSolution {
    pub items: Vec<i64>,
    pub total_value: f64,
    pub total_weight: f64,
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::bound::FractionalBound;
    use crate::prune::BoundPruning;

    #[test]
    fn simple_knapsack() {
        let ks = Knapsack::new(
            vec![10.0, 20.0, 30.0],
            vec![5.0, 10.0, 15.0],
            20.0,
        );
        let sol = ks.solve(&FractionalBound, &BoundPruning);
        // Items 0 and 2: value=40, weight=20 OR items 0 and 1: value=30, weight=15
        // Actually item 1+2: value=50, weight=25 > capacity
        // Best: item 1+2 = infeasible, item 0+2: value=40 weight=20 = capacity
        assert!(sol.total_value >= 30.0);
    }

    #[test]
    fn knapsack_exact_optimal() {
        let ks = Knapsack::new(
            vec![60.0, 100.0, 120.0],
            vec![10.0, 20.0, 30.0],
            50.0,
        );
        let sol = ks.solve(&FractionalBound, &BoundPruning);
        // Items 1+2: value=220, weight=50
        assert!((sol.total_value - 220.0).abs() < 1e-10);
    }

    #[test]
    fn knapsack_zero_capacity() {
        let ks = Knapsack::new(
            vec![10.0, 20.0],
            vec![5.0, 10.0],
            0.0,
        );
        let sol = ks.solve(&FractionalBound, &BoundPruning);
        assert!((sol.total_value - 0.0).abs() < 1e-10);
    }

    #[test]
    fn knapsack_all_fit() {
        let ks = Knapsack::new(
            vec![10.0, 20.0],
            vec![1.0, 2.0],
            100.0,
        );
        let sol = ks.solve(&FractionalBound, &BoundPruning);
        assert!((sol.total_value - 30.0).abs() < 1e-10);
    }

    #[test]
    fn knapsack_single_item() {
        let ks = Knapsack::new(vec![42.0], vec![10.0], 15.0);
        let sol = ks.solve(&FractionalBound, &BoundPruning);
        assert!((sol.total_value - 42.0).abs() < 1e-10);
    }
}
