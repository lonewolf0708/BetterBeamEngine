## 2024-05-22 - [Hot Path Optimization in Lua Simulation]
**Learning:** Defining functions (closures) and table literals inside high-frequency loops (e.g., `updateTorque` at 2000Hz) causes significant Garbage Collection pressure and performance degradation in Lua. Similarly, system calls like `os.clock()` in these loops are costly compared to using the provided `dt` (delta time).

**Action:** Move all static data structures, helper functions, and initialization logic to the module scope or the `new`/`reset` functions. Use `dt`-based accumulators for timing instead of `os.clock()`. Always ensure module-level helpers are defined above their call sites.
