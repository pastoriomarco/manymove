
# Contributing to ManyMove

Thank you for considering a contribution to **ManyMove**!
ManyMove is an experimental collection of ROS 2 packages for single or multi‑arm manipulation.
To keep the project maintainable and safe, please follow the guidelines below.

---

## Supported distributions & branch model

- **Distributions:** Humble and Jazzy.
- **Branch policy:**
  - Work happens on `dev` (kept compatible with both Humble and Jazzy).
  - Stable releases are merged from `dev` → `main`.
- **Goal:** Single codebase that builds, lint‑checks and passes tests on **both** Humble and Jazzy.

---

## Reporting issues

Open an issue on the repo for bugs, docs fixes, or feature requests. Please include:
- ROS 2 distro (Humble/Jazzy), OS, and environment details.
- Exact steps to reproduce and expected vs. actual behavior.
- Relevant logs or stack traces (use fenced code blocks).
- If it’s a feature: motivation, proposed approach, and which packages are affected.

For general Q&A, prefer **ROS Answers**.

---

## Pull Request (PR) workflow

1. **Branch from `dev`.**
   Use a descriptive name (e.g., `feature/object-manager-tests`).

2. **Design discussion (when appropriate).**
   For substantial changes, open an issue first to align on approach.

3. **Update / add tests.**
   - C++: GoogleTest (via `ament_cmake_gtest`).
   - Python: `pytest`.
   - Launch/integration: `launch_testing`.

4. **Run formatting & linters locally (pre-commit).**
   Install hooks once:
   ```bash
   pre-commit install
   ```

   Run on all files:
   ```bash
   pre-commit run --all-files
   ```

The repo uses (among others):

* **ruff** for Python style & lint
* **isort** for Python import order
* end-of-file, trailing whitespace checks, large files checks

5. **Verify C++ formatting with Uncrustify for BOTH distros.**
   The same code must pass **0.72 (Humble)** and **0.78 (Jazzy)**. For example, from `/src/manymove/` folder:

   ```bash
   # humble:
   uncrustify --check -c ./uncrustify_0.72.cfg $(git ls-files '*.cpp' '*.hpp')
   # jazzy:
   uncrustify --check -c ./uncrustify_0.78.cfg $(git ls-files '*.cpp' '*.hpp')
   ```
   To try and format the files change `--check` with `--replace --no-backup`.

6. **Ensure ament linters are integrated (so CI runs them too).**
   In each package:

   * `CMakeLists.txt` should include:

     ```cmake
     if(BUILD_TESTING)
       find_package(ament_lint_auto REQUIRED)
       ament_lint_auto_find_test_dependencies()
     endif()
     ```
   * `package.xml` should include:

     ```xml
     <test_depend>ament_lint_auto</test_depend>
     ```

7. **Build & test locally with colcon.**

   ```bash
   colcon build --symlink-install
   colcon test --event-handlers console_cohesion+
   colcon test-result --verbose
   ```

   No warnings or test failures should remain.

8. **Update versions & changelog when needed.**

   * Use **SemVer** in the affected `package.xml` files.
   * Add a concise entry to the repository‑level `CHANGELOG.rst` under **Forthcoming**.

9. **Open the PR against `dev`.**

   * Describe the problem and solution, list affected packages.
   * Link issues, mention breaking changes and migration notes if any.
   * CI must pass before merge. When possible we **Squash & Merge**.

10. **Respond to reviews** and iterate until approved.

---

## Coding standards

Follow the [ROS coding styles](https://docs.ros.org/en/humble/The-ROS2-Project/Contributing/Code-Style-Language-Versions.html).
If you notice some inconsistencies in the current codebase, please notify!

---

## Testing

* **Unit tests:** cover new logic and edge cases (GTest/pytest).
* **Integration/launch tests:** use `launch_testing` for multi‑node scenarios and launch files.
* **Cross‑distro:** validate on Humble and Jazzy.
* **CI:** GitHub Actions runs `colcon test` and linters. Keep CI green.

---

## Versioning & changelog

* **Semantic Versioning:**

  * **MAJOR**: breaking API changes
  * **MINOR**: new backwards‑compatible features
  * **PATCH**: bug fixes
* **Repository‑level changelog:**

  * `CHANGELOG.rst` at repo root is the canonical, high‑level history.
  * The **Forthcoming** section always refers to the **next merge `dev` → `main`**.
  * Group entries by package; keep them concise with links to PRs/issues.

---

## License & sign‑off

* Contributions are under **BSD‑3‑Clause**.
* Include the standard license header in new source files.

---

## Quick checklist before opening a PR

* [ ] Code builds locally (Humble & Jazzy).
* [ ] `pre-commit run --all-files` is clean.
* [ ] Uncrustify passes with **both** configs.
* [ ] `colcon test` passes; tests added/updated.
* [ ] `package.xml` versions and top‑level `CHANGELOG.rst` updated if needed.
* [ ] PR description explains the change & impact.

Thanks for helping improve ManyMove!

```

---
