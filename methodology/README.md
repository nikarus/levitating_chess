# Project organization

Keep the existing Git-tracked root file set. Supporting work belongs under `methodology/`. This is a directory guide, not a file inventory; adding a file does not require updating it.

```text
methodology/
  docs/         maintained engineering references
  tests/        shared calculation checks
  results/      retained numerical evidence
  cad/          geometry source and related tooling
    output/     generated CAD artifacts
```

Shared calculation code lives directly under `methodology/`. Add a subsystem folder only when substantial work needs its own home. Keep related source, dependencies and checks together, with generated output separate. Avoid empty folders and elaborate structures created for hypothetical work.

## Maintenance rules

- **Search before adding.** Find the existing owner of a calculation, parameter or explanation. Extend or replace it instead of creating a parallel version. Link to authoritative material rather than repeating it.
- **Keep documents about the current project.** Retain decisions in effect, essential rationale, assumptions, reproducible evidence and unresolved issues that affect the work. Do not save session transcripts, progress diaries, notes for every step, or lists of abandoned proposals.
- **Prune as part of the work.** When changing an area, also remove its obsolete guidance, completed planning detail, unused code and duplicate artifacts. Merge overlapping documents. Cleanup is part of completing a change, not a separate future project.
- **Use Git for history.** Superseded material normally leaves the working tree. Moving it into an archive is not cleanup. Keep historical material only when a current decision or regression check still depends on it; remove it when that dependency ends. Preserve unique raw measurements and evidence needed to reproduce current conclusions.
- **Organize by purpose, not chronology.** Use descriptive topic names, not session names, dates or version suffixes for maintained documents. Rearrange, consolidate or split material when that improves navigation. Split by distinct purpose, not merely because another session produced more text.
- **Keep navigation lightweight.** Use clear filenames and headings. Add a short local index only when a directory needs one; do not maintain a catalog of every file. Update affected links and consumers when moving or removing files. This README changes only when organization or its rules change.
- **Read selectively.** Follow mandatory project instructions, then locate relevant files and sections using names, headings and targeted searches. Do not load all documentation, historical reports, generated data or vendor material at the start of a session. Keep essential constraints in their authoritative location and reference them from dependent work.
- **Retain outputs deliberately.** Keep results that support current decisions, comparisons or regression checks, with their scope and source provenance. Keep temporary logs and scratch work outside the repository. Avoid accumulating copies of reproducible exports or intermediate runs.
- **Keep ownership clear.** Quantitative settings belong in `Inputs`, `Fixed` and `Constants` in `model.py`; calculations reuse shared implementations. Documentation explains results without becoming another source of settings. AI must not modify `PRODUCT_VISION.md` or `PRODUCT_TESTING.md`.

Before finishing a change, check the affected area for duplication, stale content, broken references and unnecessary files. Leave it easier to navigate, with no additional root files.
