//! # Branch and Bound Framework
//!
//! A branch and bound optimization framework with node selection strategies
//! (DFS, BFS, best-first), bounding functions, pruning, integer programming,
//! and knapsack problem solving. Zero external dependencies.
//!
//! # Example
//! ```
//! use branch_bound::{Knapsack, FractionalBound, BoundPruning};
//!
//! let ks = Knapsack::new(
//!     vec![60.0, 100.0, 120.0],
//!     vec![10.0, 20.0, 30.0],
//!     50.0,
//! );
//! let sol = ks.solve(&FractionalBound, &BoundPruning);
//! println!("Value: {}, Weight: {}", sol.total_value, sol.total_weight);
//! ```

pub mod node;
pub mod bound;
pub mod prune;
pub mod integer_prog;
pub mod knapsack;

pub use node::BNode;
pub use bound::{BoundingFunction, FractionalBound, LPRelaxationBound};
pub use prune::{PruningRule, BoundPruning, AggressivePruning, NoPruning};
pub use integer_prog::{IntegerProgram, NodeSelection};
pub use knapsack::{Knapsack, KnapsackSolution};
