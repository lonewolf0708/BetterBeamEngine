## 2025-01-24 - High-Frequency Allocation Anti-Pattern in Lua
**Learning:** In Lua-based simulation environments, defining functions (closures) or table literals inside high-frequency functions (like `updateTorque` at 2000Hz) causes significant garbage collection pressure. This creates CPU overhead and can lead to micro-stutters during simulation.
**Action:** Always hoist static data structures (lookup maps) and helper functions to the module scope if they do not depend on local loop state.
