---
id: 0006
title: Analyze Deploy Workflow Skip Issue
stage: misc
date: 2025-12-11
surface: agent
model: claude-sonnet-4-5-20250929
feature: 003-book-content-structure
branch: 003-book-content-structure
user: developer
command: /sp.implement
labels: ["github-actions", "ci-cd", "deployment", "debugging"]
links:
  spec: null
  ticket: null
  adr: null
  pr: null
files:
  - .github/workflows/deploy.yml
tests:
  - none
---

## Prompt

Read the file '.github/workflows/deploy.yml'.
Analyze why the 'deploy' job would be SKIPPED when pushing to the 'main' branch.
Check the 'on:' and 'if:' conditions.

## Response snapshot

Identified critical issue in GitHub Actions workflow causing deploy job to skip on push to main:

**Root Cause:** The `deploy` job has `needs: test-deploy` dependency (line 30), but `test-deploy` only runs on pull requests (`if: github.event_name == 'pull_request'` on line 11). When pushing to main, `test-deploy` is skipped, causing `deploy` to also skip due to the dependency requirement.

**Logic Breakdown:**
1. Push to main triggers workflow (lines 4-5: `on: push: branches: [main]`)
2. `test-deploy` job evaluates its condition (line 11) - FALSE for push events → SKIPPED
3. `deploy` job condition is TRUE (line 28), but `needs: test-deploy` requires the dependent job to succeed
4. Since `test-deploy` was SKIPPED, GitHub Actions skips `deploy` as well

**Recommended Solution:** Remove `needs: test-deploy` from the deploy job (line 30). The deploy job already has its own build and validation steps, making the dependency unnecessary.

**Alternative Solutions:**
- Make test-deploy run on both events
- Use conditional dependency with `needs.test-deploy.result` checks

## Outcome

- ✅ Impact: Identified blocker preventing GitHub Pages deployment on main branch pushes
- 🧪 Tests: Analysis only - no code changes
- 📁 Files: Analyzed .github/workflows/deploy.yml
- 🔁 Next prompts: Fix workflow by removing needs dependency or adjusting conditions
- 🧠 Reflection: GitHub Actions `needs` dependencies cause skip cascades when dependent jobs are skipped

## Evaluation notes (flywheel)

- Failure modes observed: Misunderstanding of GitHub Actions job dependency behavior with conditional execution
- Graders run and results (PASS/FAIL): PASS - Issue correctly identified with clear explanation
- Prompt variant (if applicable): none
- Next experiment (smallest change to try): Remove `needs: test-deploy` line from deploy job
