---
description: Deploy to Production (Docker Compose)
---

**Context**: This builds production containers. Run in **WSL 2** or PowerShell.

1. **Build & Spin Up**

   ```bash
   cd infra
   # Force rebuild to ensure code changes are captured
   docker compose up --build -d
   ```

2. **Verify System Health**

   ```bash
   docker ps
   # Check logs for failures
   docker compose logs -f
   ```

3. **Teardown**
   ```bash
   docker compose down
   ```
