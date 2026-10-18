# Project organization

Keep the existing Git-tracked root file set. Supporting work belongs under `methodology/`. This is a directory guide, not a file inventory; adding a file does not require updating it.

```text
methodology/
  docs/         maintained engineering references
  tests/        shared calculation checks
  results/      retained evidence and generated review artifacts
    cad/        current CAD exports
  cad/          geometry source and related tooling
```

Shared calculation code lives directly under `methodology/`. Add a subsystem folder only when substantial work needs its own home. Keep related source, dependencies and checks together, with generated output separate. Avoid empty folders and elaborate structures created for hypothetical work. The active shared Python environment is under `cad/`, beside its complete dependency lock; do not create parallel environments for each investigation.

## Maintenance rules

- **Search before adding.** Find the existing owner of a calculation, parameter or explanation. Extend or replace it instead of creating a parallel version. Link to authoritative material rather than repeating it.
- **Keep documents about the current project.** Retain decisions in effect, essential rationale, assumptions, reproducible evidence and unresolved issues that affect the work. Do not save session transcripts, progress diaries, notes for every step, or lists of abandoned proposals.
- **Prune as part of the work.** When changing an area, also remove its obsolete guidance, completed planning detail, unused code and duplicate artifacts. Merge overlapping documents. Cleanup is part of completing a change, not a separate future project.
- **Use Git for history.** Superseded material normally leaves the working tree. Moving it into an archive is not cleanup. Keep historical material only when a current decision or regression check still depends on it; remove it when that dependency ends. Preserve unique raw measurements and evidence needed to reproduce current conclusions.
- **Organize by purpose, not chronology.** Use descriptive topic names, not session names, dates or version suffixes for maintained documents. Rearrange, consolidate or split material when that improves navigation. Split by distinct purpose, not merely because another session produced more text.
- **Give each document one job.** The remaining plan owns execution order; maintained references own architecture, current engineering evidence or test coverage. Link between these roles instead of copying their contents. Do not retain a narrative paragraph for every completed run.
- **Keep navigation lightweight.** Use clear filenames and headings. Add a short local index only when a directory needs one; do not maintain a catalog of every file. Update affected links and consumers when moving or removing files. This guide changes only when organization or its rules change.
- **Read selectively.** Follow mandatory project instructions, then locate relevant files and sections using names, headings and targeted searches. Do not load all documentation, historical reports, generated data or vendor material at the start of a session. Keep essential constraints in their authoritative location and reference them from dependent work.
- **Use one output tree.** Generated artifacts belong under `results/`, never beside source code. Keep a trial's report, trace, necessary checkpoints and optional replay together. CAD review exports belong in `results/cad/`. New studies use a purpose-named result folder; do not rearrange old numerical evidence merely for cosmetic consistency.
- **Retain outputs deliberately.** Keep results that support current decisions, comparisons or regression checks, with their scope and source provenance. Preserve failed numerical evidence when it explains a current limitation. Remove superseded review exports and videos when their underlying evidence remains available. Ignore rebuildable field caches, bytecode and application backups; keep temporary logs and scratch work outside the repository.
- **Preserve evidence identity.** Saved reports and checkpoints describe their executed source/configuration, even after files move. Never rewrite recorded paths or hashes to make old evidence appear current. Retain exact source snapshots when they are needed for reproduction. Checkpoint resume and cached-field guards remain strict; use matching archived sources or start a fresh run after source changes. Intentional reference/repeat records may be identical and still serve different verification roles.
- **Keep ownership clear.** Quantitative settings belong in `Inputs`, `Fixed` and `Constants` in `model.py`; calculations reuse shared implementations. Documentation explains results without becoming another source of settings. AI must not modify `PRODUCT_VISION.md` or `PRODUCT_TESTING.md`.

Before finishing a change, check the affected area for duplication, stale content, broken references and unnecessary files. Leave it easier to navigate, with no additional root files.
