# Technical report (Phase 1)

**Limit: 6 pages in total.** Submission deadline: 05/10/2026.

Each section is its own file in `sections/`, and its owner is named on the
first line. Edit only your own file, to avoid merge conflicts. Put images in
`figures/` and include them with `\includegraphics{name}`.

Build the PDF (needs a TeX distribution, e.g. MacTeX or TeX Live):

```sh
cd docs/report && latexmk -pdf main.tex
```

No LaTeX installed? Upload this folder to Overleaf.

`\todo{...}` prints a red placeholder. None may remain in the submitted PDF:
`grep -rn '\\todo' sections/`.
