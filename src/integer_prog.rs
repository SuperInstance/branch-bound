//! Integer programming solver using branch and bound.

use crate::node::BNode;
use crate::bound::BoundingFunction;
use crate::prune::PruningRule;

/// Node selection strategy.
#[derive(Clone)]
pub enum NodeSelection {
    /// Depth-first search.
    DFS,
    /// Breadth-first search.
    BFS,
    /// Best-first (highest bound).
    BestFirst,
}

/// Integer programming problem.
pub struct IntegerProgram {
    /// Objective coefficients (maximize c^T x).
    pub coefficients: Vec<f64>,
    /// Constraint matrix (Ax <= b).
    pub constraints: Vec<(Vec<f64>, f64)>,
    /// Variable bounds (lower, upper) for each variable.
    pub var_bounds: Vec<(i64, i64)>,
}

impl IntegerProgram {
    /// Create a simple binary integer program.
    pub fn binary(coefficients: Vec<f64>, constraints: Vec<(Vec<f64>, f64)>) -> Self {
        let n = coefficients.len();
        let var_bounds = vec![(0, 1); n];
        Self { coefficients, constraints, var_bounds }
    }

    /// Solve using branch and bound.
    pub fn solve<B: BoundingFunction, P: PruningRule>(
        &self,
        bounding: &B,
        pruning: &P,
        selection: NodeSelection,
    ) -> Option<(Vec<i64>, f64)> {
        let n = self.coefficients.len();
        let mut best_solution: Option<Vec<i64>> = None;
        let mut best_value = f64::NEG_INFINITY;

        let root = BNode::new(0, vec![], 0.0);
        let mut open = vec![root];

        while !open.is_empty() {
            let node = match selection {
                NodeSelection::DFS => open.pop().unwrap(),
                NodeSelection::BFS => open.remove(0),
                NodeSelection::BestFirst => {
                    let idx = open.iter().enumerate()
                        .max_by(|(_, a), (_, b)| a.bound.partial_cmp(&b.bound).unwrap())
                        .unwrap().0;
                    open.remove(idx)
                }
            };

            // Compute bound
            let weights: Vec<f64> = self.constraints.first().map(|c| c.0.clone()).unwrap_or_else(|| vec![1.0; n]);
            let cap = self.constraints.first().map(|c| c.1).unwrap_or(f64::MAX);
            let bound = bounding.lower_bound(&node.partial, &self.coefficients, &weights, cap);

            // Check feasibility
            let is_feasible = bound != f64::NEG_INFINITY;

            if pruning.should_prune(bound, best_value, is_feasible) {
                continue;
            }

            // Check if complete
            if node.is_complete(n) {
                let solution: Vec<i64> = node.partial.iter().map(|v| v.unwrap()).collect();
                let value = self.eval_solution(&solution);
                if value > best_value && self.is_feasible(&solution) {
                    best_value = value;
                    best_solution = Some(solution);
                }
                continue;
            }

            // Branch on next unfixed variable
            let next_var = node.partial.len();
            if next_var < n {
                let (left, right) = node.branch(next_var, n);
                let mut left = left;
                left.bound = bound;
                let mut right = right;
                right.bound = bound;
                open.push(left);
                open.push(right);
            }
        }

        best_solution.map(|s| (s, best_value))
    }

    fn eval_solution(&self, sol: &[i64]) -> f64 {
        self.coefficients.iter().zip(sol).map(|(c, v)| c * *v as f64).sum()
    }

    fn is_feasible(&self, sol: &[i64]) -> bool {
        for (row, rhs) in &self.constraints {
            let val: f64 = row.iter().zip(sol).map(|(c, v)| c * *v as f64).sum();
            if val > *rhs + 1e-10 {
                return false;
            }
        }
        for (i, v) in sol.iter().enumerate() {
            if *v < self.var_bounds[i].0 || *v > self.var_bounds[i].1 {
                return false;
            }
        }
        true
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::bound::FractionalBound;
    use crate::prune::BoundPruning;

    #[test]
    fn solve_simple_binary() {
        // Maximize 3x1 + 2x2 subject to 2x1 + x2 <= 4
        let ip = IntegerProgram::binary(
            vec![3.0, 2.0],
            vec![(vec![2.0, 1.0], 4.0)],
        );
        let result = ip.solve(&FractionalBound, &BoundPruning, NodeSelection::DFS);
        let (sol, val) = result.unwrap();
        assert!((val - 5.0).abs() < 1e-10); // x1=1, x2=1 -> 3+2=5
        assert!(sol.iter().all(|&v| v == 0 || v == 1));
    }

    #[test]
    fn solve_bfs() {
        let ip = IntegerProgram::binary(
            vec![5.0, 4.0],
            vec![(vec![3.0, 2.0], 6.0)],
        );
        let result = ip.solve(&FractionalBound, &BoundPruning, NodeSelection::BFS);
        let (_, val) = result.unwrap();
        assert!((val - 9.0).abs() < 1e-10); // x1=1, x2=1: 5+4=9, 3+2=5 <= 6
    }

    #[test]
    fn solve_best_first() {
        let ip = IntegerProgram::binary(
            vec![5.0, 4.0],
            vec![(vec![3.0, 2.0], 6.0)],
        );
        let result = ip.solve(&FractionalBound, &BoundPruning, NodeSelection::BestFirst);
        let (_, val) = result.unwrap();
        assert!((val - 9.0).abs() < 1e-10); // x1=1, x2=1: 5+4=9, 3+2=5 <= 6
    }

    #[test]
    fn solve_infeasible() {
        let ip = IntegerProgram::binary(
            vec![10.0],
            vec![(vec![5.0], 1.0)], // 5x1 <= 1, x1 in {0,1} -> only x1=0
        );
        let result = ip.solve(&FractionalBound, &BoundPruning, NodeSelection::DFS);
        let (_, val) = result.unwrap();
        assert!((val - 0.0).abs() < 1e-10);
    }
}
