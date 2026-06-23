# flask_website

Personal Flask website with technical write-ups on estimation, control, and coding, plus portfolio and resume pages. Deployed to [PythonAnywhere](https://www.pythonanywhere.com/).

## Quick start

Requires Python 3.12 and [uv](https://docs.astral.sh/uv/).

```bash
uv sync           # install deps into .venv
uv run app.py     # start dev server on http://127.0.0.1:8080
```

Add a dependency with `uv add <pkg>`.

## Sections

- **Estimation** — Kalman filter notes (CKF, SCKF), DC motor estimation, applications.
- **Control** — LQR for DC motor and pendulum, MATLAB command reference.
- **Coding** — OOP notes.
- **Portfolio / Resume** — personal pages.

Math is rendered with MathJax (loaded in `templates/base.html`), so content pages can use `\(...\)` and `$$...$$` inline.

## Project layout

```
app.py              # routes + dynamic URL registration
pyproject.toml      # project metadata + dependencies
uv.lock             # locked dependency versions
static/             # css, js, images, docs, code
templates/
  base.html         # layout, nav, MathJax
  estimation/  control/  coding/
```

## Adding a page

1. Create `templates/<section>/<name>.html` extending `base.html`.
2. Register it in the matching `routes_<section>` dict in `app.py`:
   ```python
   '/<section>/<name>': {'template': './<section>/<name>.html', 'endpoint': '<name>'}
   ```
3. Link to it from other templates with `{{ url_for('<endpoint>') }}`.

See [`CLAUDE.md`](./CLAUDE.md) for architecture notes and implementation gotchas.
