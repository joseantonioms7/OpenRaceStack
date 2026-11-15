## Summary of Changes
<!-- Provide a clear, high-level summary of what this PR changes and why. Include references to relevant issues. -->
Closes #

## Type of Change
- [ ] 🐛 **Bug fix** (non-breaking fix for an algorithmic or runtime issue)
- [ ] ✨ **New feature** (new estimator, controller, scenario, or dashboard enhancement)
- [ ] 🚀 **Performance optimization** (memory footprint reduction, zero-allocation vectorization)
- [ ] 📚 **Documentation** (mathematical derivations, tutorials, API references)
- [ ] 🔧 **Tooling / Packaging / CI** (pre-commit, packaging, GitHub Actions)
- [ ] 💥 **Breaking change** (interface change requiring downstream code updates)

## Mathematical & Algorithmic Verification
- [ ] **Positive Definiteness:** Error covariance matrices remain strictly Symmetric Positive Definite ($P = P^T \succ 0$).
- [ ] **Manifold Correctness:** Angular errors and heading states are wrapped on the principal branch $[-\pi, \pi)$.
- [ ] **Benchmark Integrity:** 2D Position RMSE on Monza centerline does not regress ($\le 0.35\,\text{m}$).
- [ ] **Regulatory KPIs:** Automated 7-rule KPI scorecard in `SessionOrchestrator` passes nominal thresholds.
- [ ] **Zero Oracle Leakage:** Estimators do not read ground-truth simulation arrays directly.

## Quality & Code Standards
- [ ] Code adheres to PEP 8 standards and passes `ruff check .` and `ruff format --check .`.
- [ ] Type annotations included and verified with `mypy Localization/`.
- [ ] Unit tests added in `tests/` covering the new behavior.
- [ ] Docstrings updated with LaTeX mathematical notation where appropriate.
- [ ] All code comments, docstrings, and commit messages are in English.
