# Glass Lab Automation

This repository contains the codebase for our VT3 project, **Glass Lab Automation**.

The purpose of the project is to develop and integrate the different parts of the automated Glass Lab setup, including the oven, MiR, robot manipulator, dispenser, and communication between the subsystems.

---

## Repository Structure

The repository is divided into the main parts of the system:

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

### Folders

- `oven/`  
  Code related to oven control, actuator control, and temperature button interaction.

- `mir/`  
  Code related to MiR communication and control.

- `robot/`  
  Code for the robot manipulator.

- `dispenser/`  
  Code for the precursor dispenser.

- `integration/`  
  Code used to connect and coordinate the different subsystems.

- `docs/`  
  Technical notes, documentation, test procedures, and other project-related material.

---

# Getting Started

## 1. Clone the repository

```bash
git clone <repository-url>
cd glass-lab-automation
```

## 2. Install dependencies

The Python part of the project uses `uv`.

```bash
uv sync
```

This creates the environment and installs the dependencies defined in the project.

## 3. Run Python code

Use:

```bash
uv run python <file.py>
```

Example:

```bash
uv run python mir/test_connection.py
```

---

# GitHub Project

Development tasks are managed in the GitHub Project:

**VT3 - Glass Lab Automation**

Each task should be created as an issue and added to the project board.

The project uses the following fields:

### Status

```text
Backlog
Ready
In Progress
Testing
Done
```

### MVP

```text
MVP1
MVP2
MVP3
MVP4
```

### Subsystem

```text
Oven
MiR
Robot
Dispenser
Integration
```

### Priority

```text
High
Medium
Low
```

These fields make it easier to see what is being worked on and which part of the project each issue belongs to.

---

# Working Rules

For each task, use the following workflow.

## 1. Pick an issue

Choose an issue from the GitHub Project.

Make sure the issue has the correct:

- MVP
- Subsystem
- Priority
- Assignee

## 2. Move the issue to `In Progress`

When you start working on an issue, change its status to:

```text
In Progress
```

## 3. Create a branch

Do **not** work directly on `main`.

Start by updating your local `main`:

```bash
git checkout main
git pull
```

Then create a new branch:

```bash
git checkout -b feature/name-of-feature
```

Example:

```bash
git checkout -b feature/mir-connection
```

## 4. Do the work

Make the changes related to the issue.

Try to keep each branch focused on one task.

## 5. Commit your changes

Add the changed files:

```bash
git add .
```

Commit them:

```bash
git commit -m "Add MiR connection test"
```

## 6. Push the branch

```bash
git push -u origin feature/mir-connection
```

## 7. Open a Pull Request

Create a Pull Request from your branch into:

```text
main
```

The Pull Request should briefly explain:

- What was changed
- Why it was changed
- How it was tested

## 8. Review

At least one other group member should review the Pull Request.

If comments or problems are found, fix them before merging.

## 9. Merge into `main`

When the Pull Request has been reviewed and the code is ready, merge it into `main`.

Then update your local repository:

```bash
git checkout main
git pull
```

## 10. Update the issue

After the code has been merged:

```text
Testing
```

should be used if further physical or integration testing is still needed.

Move the issue to:

```text
Done
```

when the task is completely finished.

---

# Branch Naming

Use clear branch names.

### New functionality

```text
feature/oven-door-control
feature/mir-connection
feature/robot-pickup
feature/dispenser-control
```

### Bug fixes

```text
fix/mir-timeout
fix/oven-door-command
```

### Documentation

```text
docs/update-readme
docs/add-test-procedure
```

Keep branch names short and descriptive.

---

# Commit Guidelines

Commit messages should explain what was changed.

Good examples:

```text
Add MiR connection test
Implement oven open command
Fix timeout handling
Update MVP1 test procedure
```

Avoid unclear commit messages such as:

```text
stuff
changes
test
fix
new
```

Commits do not need to be perfect, but someone else should be able to understand what changed from the message.

---

# Pull Request Rules

Before merging a Pull Request:

- The code should work as intended.
- The code should be related to the issue being worked on.
- At least one other group member should review it.
- Review comments should be resolved.
- Do not merge unfinished or knowingly broken code into `main`.

If something is experimental and not ready for `main`, keep it on the feature branch.

---

# Definition of Done

An issue can be moved to `Done` when:

- The required functionality has been implemented.
- The code has been tested.
- The code has been reviewed.
- The Pull Request has been merged into `main`.
- Any required physical or integration testing has been completed.
- Relevant documentation has been updated if necessary.

If the code has been merged but still needs physical testing, move the issue to:

```text
Testing
```

instead of `Done`.

---

# Dependencies

Python dependencies should be added using `uv`.

Example:

```bash
uv add requests
```

This updates:

```text
pyproject.toml
uv.lock
```

Both files should be committed to Git.

Other group members can then update their environment with:

```bash
uv sync
```

This helps keep everyone on the same dependency versions.

---

# Secrets and Configuration

Do not commit sensitive information to GitHub.

Examples include:

- Passwords
- API keys
- Robot credentials
- Private tokens
- Wi-Fi credentials

If configuration values are needed locally, use an `.env` file or another local configuration file that is ignored by Git.

Example `.gitignore` entry:

```gitignore
.env
```

---

# Important Rule

The most important Git rule for this project is:

> **Do not work directly on `main`.**

Use:

```text
Issue → Branch → Code → Pull Request → Review → Merge
```

This keeps the main branch stable and makes it easier for everyone in the group to work on the same codebase.
