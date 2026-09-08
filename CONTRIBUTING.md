# Contributing to Driverless

Thank you for contributing to the ARTTU Formula Student Driverless project.

## Development workflow

1. Create or switch to a feature branch from `dev`.
2. Make focused changes with clear commit messages.
3. Run the test suite locally:

```bash
python -m pytest -q
```

4. Open a pull request against `dev`.
5. Keep the PR focused and document engineering assumptions and validation results.

## Branching model

- `main` — stable, reviewed project state.
- `dev` — integration/development branch.
- Feature branches — individual pieces of work, merged into `dev` through pull requests.

## Engineering changes

For vehicle-model changes, document:

- parameter sources and units;
- assumptions or temporary values;
- equations/model changes;
- validation data and expected results;
- known limitations.

Measured vehicle data should be distinguished clearly from assumptions and simulation outputs.

## Generated files

Do not commit Python caches, virtual environments, IDE metadata, local telemetry, raw datasets, MATLAB temporary files, or generated simulation output unless the output is intentionally part of the project record.

## Commit messages

Prefer concise, descriptive messages using conventional prefixes where practical:

- `feat:` new functionality
- `fix:` bug fix
- `test:` tests
- `docs:` documentation
- `refactor:` code restructuring
- `ci:` CI/CD changes
- `chore:` maintenance

## Pull requests

PRs should explain the motivation, summarize the changes, and state how the changes were validated. For model changes, include relevant numerical results and comparisons with measured vehicle data when available.
