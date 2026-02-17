# How to Think: Architectural Decisions - Aliases vs. Explicit Commands

## The Decision: Alias vs. Workflow

**Verdict**: We do **NOT** rely on local shell aliases for core project tasks.
**Reasoning**:

1.  **Portability**: If I create an alias `run-all` in your `.bashrc`, it works today. If you switch machines or a new developer joins, the command vanishes. The project breaks.
2.  **Transparency**: Hidden logic is dangerous. A command like `npm run build` is explicit and documented in `package.json`. An alias is a black box.
3.  **CI/CD**: Your build server (GitHub Actions/GitLab CI) does not have your personal alias file. It needs standard commands.

## The Professional Alternative: Project Scripts

Instead of personal aliases, we use **Standardized Entry Points**.

| Task             | Personal Alias (Bad) | Standard Script (Professional)                 |
| :--------------- | :------------------- | :--------------------------------------------- |
| Build Everything | `b`                  | `npm run build:all` (in root `package.json`)   |
| Start Dev        | `dev`                | `npm run dev` (uses Turbo/Concurrently)        |
| Launch ROS       | `ros`                | `ros2 launch ...` (Shell script in `scripts/`) |

## Exception: Personal Convenience

You (the user) are free to alias `g` to `git` or `k` to `kubectl` in your own shell profile. But **Antigravity (The Agent)** will always use the full, explicit command to ensure reproducibility and clarity in logs.

## Implemented Strategy

1.  **Root `package.json`**: We will create a root `package.json` to act as the "Master Control" for the monorepo.
2.  **Helper Scripts**: Complex logic (like "start 5 terminals") goes into `./scripts/dev.sh`, which is checked into Git.
