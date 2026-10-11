#!/bin/sh
# Look up the code the car ran from the AiM Version Hi/Lo channels.
#   tools/find_build.sh <version_hi> <version_lo>

if [ $# -ne 2 ]; then
    echo "usage: $0 <version_hi> <version_lo>" >&2
    exit 1
fi

cd "$(dirname "$0")/.." || exit 1

short=$(printf '%04x%04x' "$1" "$2") || exit 1
if [ "$short" = "00000000" ]; then
    echo "Hash 0: build had no version info"
    exit 1
fi

git fetch --quiet origin flash-log 2>/dev/null

full=$(git rev-parse --verify --quiet "$short^{commit}")
if [ -z "$full" ]; then
    echo "$short: not found (fetch flash-log, or it was never pushed)"
    exit 1
fi

git log -1 --format='%H%n%s%n%b' "$full"

case "$(git log -1 --format=%s "$full")" in
flash\ *)
    base=$(git rev-parse -q --verify "$full^2" || git rev-parse -q --verify "$full^1")
    if [ -n "$base" ] && [ -n "$(git diff --name-only "$base" "$full")" ]; then
        echo "Uncommitted changes on top of $(git rev-parse --short "$base"):"
        git diff --stat "$base" "$full"
    fi
    ;;
esac

url=$(git remote get-url origin 2>/dev/null |
    sed -e 's#^git@github.com:#https://github.com/#' -e 's#\.git$##')
[ -n "$url" ] && echo && echo "$url/commit/$full"
exit 0
