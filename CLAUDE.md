# CLAUDE.md

Personal Flask website with technical content sections (estimation, control, coding) plus portfolio/resume pages. Deployed to PythonAnywhere.

## Run

```
uv sync           # one-time: install deps into .venv (Python 3.12)
uv run app.py     # start the dev server
```

Serves on `127.0.0.1:8080` with `debug=True` (see `app.py:233-234`). No separate dev/prod config — the `__main__` block is the entry point.

Add a new dependency with `uv add <pkg>` (updates `pyproject.toml` and `uv.lock`). Activate the venv interactively with `source .venv/bin/activate` if you prefer running `python app.py` directly.

## Architecture

Routes are registered **dynamically** from dictionaries, not via `@app.route` decorators. Each section has its own `routes_<section>` dict in `app.py`, and `route_2_render_template()` walks each dict and calls `app.add_url_rule()`.

To add a new page:
1. Create the template under `templates/<section>/<name>.html` extending `base.html`.
2. Add an entry to the matching `routes_<section>` dict in `app.py`:
   ```python
   '/<section>/<name>': {'template': './<section>/<name>.html', 'endpoint': '<name>'}
   ```
3. Reference it in other templates via `{{ url_for('<endpoint>') }}` — note `url_for` takes the **endpoint**, not the URL path.

### Late-binding lambda — don't break this

`route_2_render_template` uses `lambda template=data['template']: render_template(template)`. The default-argument trick captures `template` per-iteration. Refactoring to `lambda: render_template(data['template'])` would silently break all routes (every URL would render whatever the *last* iteration's template was) — Python closures bind by reference, not value.

## Templates

- All pages extend `templates/base.html` via Jinja2 `{% extends %}` / `{% block content %}`.
- MathJax is loaded in `base.html` for math equation rendering — content pages can use `\(...\)` and `$$...$$` directly.
- Section landing pages (`estimation.html`, `control.html`, `coding.html`) are tables-of-contents that link into subdirectory templates.

## Conventions and gotchas

- **No contact form** — removed in the 12/8/24 refactor. Don't reintroduce `flask_mail` / SMTP code; `contact.html` and the contact route are intentionally gone from nav.
- **Dead commented-out routes** at the bottom of `app.py` (lines ~128–227) are leftover from the decorator-style refactor. Safe to delete; left in for now as a reference.
- **`.idea/workspace.xml` is still tracked** despite being in `.gitignore` — Git only ignores untracked files. Use `git rm --cached -r .idea` to fully untrack when convenient.

## Layout

```
app.py              # routes dicts + dynamic registration
pyproject.toml      # project metadata + Flask dependency
uv.lock             # locked transitive deps (commit this)
.venv/              # local virtualenv (gitignored)
static/
  css/  js/  images/  docs/  code/
templates/
  base.html         # layout + nav + MathJax
  index.html  portfolio.html  resume.html
  estimation/       # intro, ckf, sckf, estimate_dc_motor, application_1/2, reference
  control/          # lqr_for_dc_motor, lqr_for_pendulum, matlab_commands
  coding/           # oop
```
