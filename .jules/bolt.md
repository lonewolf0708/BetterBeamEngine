## 2024-04-19 - Optimization of cold start enrichment hot path
**Learning:** In high-frequency loops (like `updateTorque` running at 2000Hz), defining functions and tables locally causes excessive garbage collection pressure due to constant allocations.
**Action:** Move static data structures and helper functions to the module level to avoid re-allocation in the hot path.
