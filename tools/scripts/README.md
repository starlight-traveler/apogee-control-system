# Utility Scripts

Small repository maintenance commands live here. These scripts should be stable enough to run from the repository root or from their own directory.

## Doxygen API Docs

Generate source-level API documentation with:

```bash
bash tools/scripts/generate_doxygen.sh
```

The script reads [`docs/Doxyfile`](../../docs/Doxyfile) and writes generated HTML to `docs/api/html/`. That directory is ignored by git because it is a rebuildable artifact.
