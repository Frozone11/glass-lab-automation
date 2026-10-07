# Glass Lab Automation

This repository contains the codebase for our VT3 project, **Glass Lab Automation**.

The project includes code for the oven, MiR, robot manipulator, dispenser, and integration between the different subsystems.

## Repository Structure

```text
glass-lab-automation/
├── oven/
├── mir/
├── robot/
├── dispenser/
├── integration/
├── docs/
├── pyproject.toml
├── uv.lock
└── README.md
```

- `oven/` - Oven control and actuators
- `mir/` - MiR communication and control
- `robot/` - Robot manipulator code
- `dispenser/` - Dispenser code
- `integration/` - Communication between subsystems
- `docs/` - Documentation and test material

---

# Setup

Clone the repository:

```bash
git clone <repository-url>
cd glass-lab-automation
```

Install dependencies:

```bash
uv sync
```

Run Python code with:

```bash
uv run python <file.py>
```

---

# Working Workflow

The main rule is:

> **Do not work directly on `main`.**

Each task should follow this workflow:

```text
Issue
  ↓
Move to In Progress
  ↓
Create Branch
  ↓
Write Code
  ↓
Commit + Push
  ↓
Pull Request
  ↓
Review
  ↓
Merge into main
  ↓
Testing / Done
```

## 1. Pick an Issue

Choose a task from the GitHub Project and assign it to yourself.

Move the issue to:

```text
In Progress
```

## 2. Create a Branch

Always create the branch from an updated `main`:

```bash
git switch main
git pull
git switch -c feature/name-of-feature
```

Example:

```bash
git switch -c feature/oven-door
```

Use clear branch names such as:

```text
feature/oven-door
feature/mir-connection
fix/mir-timeout
docs/update-readme
```

## 3. Work and Commit

Make your changes and then:

```bash
git add .
git commit -m "Implement oven door control"
```

Commit messages should briefly describe what was changed.

## 4. Push the Branch

The first time you push a new branch:

```bash
git push -u origin feature/oven-door
```

After that, you can normally use:

```bash
git push
```

## 5. Create a Pull Request

On GitHub, create a Pull Request from your branch into:

```text
main
```

Briefly describe:

- What was changed
- How it was tested

Link the Pull Request to the issue using:

```text
Closes #<issue-number>
```

Example:

```text
Closes #2
```

## 6. Review and Merge

At least one other group member must review and approve the Pull Request.

Resolve any comments before merging.

Once approved, merge the Pull Request into `main`.

Do not merge unfinished or knowingly broken code.

## 7. Finish the Issue

After merging:

- Move the issue to `Testing` if physical or integration testing is still needed.
- Move it to `Done` when the task is fully completed.

---

# Dependencies

Add Python dependencies using:

```bash
uv add <package>
```

Example:

```bash
uv add requests
```

Other group members can then run:

```bash
uv sync
```

to get the same dependencies.

---

# Important Rules

- Do not work directly on `main`.
- One branch should normally represent one task.
- Pull Requests must be reviewed before merging.
- Pull the newest `main` before creating a new branch.
- Do not commit passwords, API keys, tokens, or other secrets.
