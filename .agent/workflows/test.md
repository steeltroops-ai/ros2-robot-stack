---
description: Comprehensive Test Suite (Bun + UV)
---

1. **Unit Tests (Fast)**

   ```bash
   # Shared
   cd packages/shared-types && bun test

   # Backend
   cd apps/backend && bun test

   # Frontend
   cd apps/frontend && bun test
   ```

2. **Type Checking (Static Analysis)**

   ```bash
   # TypeScript Composite check
   turbo run type-check
   # OR individual
   cd apps/frontend && bun run type-check
   ```

3. **Python Lint/Test**

   ```bash
   cd apps/ml-service
   source .venv/bin/activate
   flake8 .
   pytest
   ```

4. **ROS 2 Tests**
   ```bash
   cd robotics/ros2_ws
   colcon test
   colcon test-result --verbose
   ```
