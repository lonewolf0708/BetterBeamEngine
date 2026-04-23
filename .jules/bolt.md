## 2024-05-22 - [Closure and Table Hoisting in Hot Paths]
**Learning:** Defining functions (closures) and table literals inside high-frequency functions (like `updateTorque` at 2000Hz) causes significant garbage collection pressure and performance degradation due to constant re-allocation.
**Action:** Move static data structures (like lookup maps) and helper functions to the module scope or initialize them once in the constructor (`new` function).

## 2024-05-22 - [Syscall Reduction in Hot Paths]
**Learning:** System calls like `os.clock()` are expensive compared to local variable access and can introduce measurable overhead when called thousands of times per second.
**Action:** Use the provided simulation `dt` (delta time) for timing logic in hot paths instead of querying the system clock.
