---
description: Lint all codebases
---

1. Lint Frontend

   ```
   cd apps/frontend && bun run lint
   ```

2. Lint Backend

   ```
   cd apps/backend && bun run lint
   ```

3. Lint ML Service (flake8/black)

   ```
   cd apps/ml-service && flake8 .
   ```

4. Lint C++ (ament_lint)
   ```
   cd robotics/ros2_ws
   ament_lint_auto src/
   ```