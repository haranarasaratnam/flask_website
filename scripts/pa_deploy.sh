#!/usr/bin/env bash
set -euo pipefail

REPO="${HOME}/flask_website"
WSGI="/var/www/${USER}_pythonanywhere_com_wsgi.py"

cd "$REPO"
git fetch --quiet origin
git pull --ff-only origin main
touch "$WSGI"
echo "[$(date -u +%FT%TZ)] deployed $(git rev-parse --short HEAD)"
