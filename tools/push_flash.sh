#!/bin/sh
# Push the flash-log branch to GitHub, joining in builds teammates pushed.
# Logged hashes are never rewritten, histories are merged instead.

cd "$(dirname "$0")/.." || exit 1
export GIT_TERMINAL_PROMPT=0

LOG=refs/heads/flash-log

local=$(git rev-parse -q --verify "$LOG") || exit 0
git fetch --quiet origin flash-log 2>/dev/null
remote=$(git rev-parse -q --verify refs/remotes/origin/flash-log)

if [ -n "$remote" ] && ! git merge-base --is-ancestor "$remote" "$local"; then
    if git merge-base --is-ancestor "$local" "$remote"; then
        git update-ref "$LOG" "$remote" "$local" || exit 1
        exit 0
    fi
    joined=$(git commit-tree "$local^{tree}" -p "$local" -p "$remote" \
        -m "merge flash-log $(date '+%Y-%m-%d %H:%M:%S')") || exit 1
    git update-ref "$LOG" "$joined" "$local" || exit 1
fi

git push --quiet origin "$LOG:$LOG"
