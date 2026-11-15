# Contributing to ApexPathAI

Thank you for your interest in contributing to ApexPathAI! We welcome contributions from autonomous driving researchers, robotics engineers, control theorists, and open-source software developers.

---

## 1. Code of Conduct

All contributors and maintainers are expected to abide by our [Code of Conduct](CODE_OF_CONDUCT.md) (Contributor Covenant v2.1). Please read it to ensure a respectful and welcoming environment for everyone.

---

## 2. Development Setup

### Prerequisites
- Python 3.11, 3.12, or 3.13
- Node.js 18+ and npm (for frontend dashboard development)
- Docker & Docker Compose (optional, for containerized local workflows)
- Git for version control

### Step-by-Step Installation

#### Backend Setup
```bash
# 1. Clone the repository
git clone https://github.com/apexpath-ai/ApexPathAI.git
cd ApexPathAI

# 2. Create and activate a virtual environment
python -m venv .venv
source .venv/bin/activate       # On Linux / macOS
# or: .venv\Scripts\activate    # On Windows

# 3. Install in editable mode with development, testing, and docs dependencies
pip install -e ".[dev,test,docs]"

# 4. Run automated test suite
pytest tests/ -v
```

#### Frontend Setup
```bash
cd frontend

# Install dependencies and start Vite dev server
npm install
npm run dev
```

---

## 3. Code Standards & Tooling

To maintain automotive-grade software reliability, we enforce strict formatting and static typing across all modules:

### Linters & Formatters
We use [Ruff](https://astral.sh/ruff) for linting and formatting.
```bash
# Check code style and common bugs
ruff check .

# Automatically apply safe fixes
ruff check --fix .

# Verify formatting
ruff format --check .
```

### Static Type Checking
We use [Mypy](https://mypy-lang.org/) with strict type inference on core modules:
```bash
mypy Localization/
```

### Documentation & Language Standard
- **English Standard:** All code comments, docstrings, variable names, commit messages, and PR discussions must be written strictly in English.
- **Mathematical Docstrings:** Algorithmic functions should include LaTeX mathematical representations of equations, matrices, and state vectors (e.g. `$x = [p_x, p_y, \theta, v, \omega]^T$`).

---

## 4. Git Branching & Commit Conventions

### Branch Naming Conventions
- `feature/<short-description>`: New estimators, controllers, sensors, or UI screens.
- `fix/<short-description>`: Bug fixes, numerical stability patches, or threshold adjustments.
- `docs/<short-description>`: Documentation, math guides, and tutorials.
- `perf/<short-description>`: Computational speedups or zero-allocation vectorizations.

### Commit Messages (Conventional Commits)
Please follow the [Conventional Commits v1.0.0](https://www.conventionalcommits.org/) specification:
- `feat: add 3D LiDAR surface normal point-to-plane residual`
- `fix: correct angle wrapping on so(3) rotation injection`
- `perf: pre-allocate Joseph covariance workspace buffers`
- `docs: derive analytical Jacobians for CTRV circular arc`
- `test: add unit tests for Mahalanobis chi2 outlier gate`

---

## 5. Algorithmic & Testing Requirements

Before opening a pull request that alters estimation or control logic:
1. **Covariance Symmetry & Positive-Definiteness:** Filter updates must guarantee $P = P^T \succ 0$.
2. **Benchmark Non-Regression:** Run the 23-track evaluation suite:
   ```bash
   python -m Localization.main_train_test
   ```
   Verify that 2D Position RMSE on Monza centerline does not regress ($\le 0.35\,\text{m}$).
3. **Automated KPI Validation:** Verify that the 7-rule KPI scorecard in `SessionOrchestrator` passes.
4. **Unit Tests:** Add corresponding tests in `tests/` verifying nominal behavior, edge cases, and failure modes.

---

## 6. Pull Request Submission

1. Push your branch to your GitHub fork:
   ```bash
   git push origin feature/my-new-feature
   ```
2. Open a Pull Request against the `main` branch of `ApexPathAI`.
3. Complete the [Pull Request Template](.github/PULL_REQUEST_TEMPLATE.md) checklist.
4. Ensure all CI checks (linting, type checking, test matrix) pass.
5. Address maintainer feedback promptly. Once approved by designated [CODEOWNERS](.github/CODEOWNERS), your PR will be squash-merged into `main`.

Thank you for contributing to the future of open-source autonomous vehicle localization!
