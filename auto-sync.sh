#!/bin/bash
# Auto-commit and push horus/ changes to main and dev
# Runs via cron every hour

cd /home/neos/softmata/horus || exit 1

# Check for changes
if [ -z "$(git status --porcelain)" ]; then
    exit 0
fi

# Stage, commit, push
git add -A
git commit -m "auto: hourly sync $(date '+%Y-%m-%d %H:%M')"
git push origin main
git push origin main:dev
