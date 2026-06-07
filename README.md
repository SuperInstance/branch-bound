# branch-bound

A branch and bound optimization framework in Rust with zero external dependencies.

## Features

- **Node Selection**: DFS, BFS, and best-first strategies
- **Bounding**: Fractional relaxation and LP relaxation bounds
- **Pruning**: Bound-based, aggressive, and no-pruning strategies
- **Integer Programming**: Binary integer program solver
- **Knapsack**: Complete 0/1 knapsack problem solver

## Usage

```rust
use branch_bound::{Knapsack, FractionalBound, BoundPruning};

let ks = Knapsack::new(
    vec![60.0, 100.0, 120.0],
    vec![10.0, 20.0, 30.0],
    50.0,
);
let sol = ks.solve(&FractionalBound, &BoundPruning);
println!("Value: {}, Weight: {}", sol.total_value, sol.total_weight);
```

## License

MIT
