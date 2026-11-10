#!/bin/sh
# List the branches of the remote that are fully merged into its main branch.
# Read-only: it asks the remote for its branch heads (git ls-remote), compares them with the
# commits this clone already has, and prints the command that would delete the merged ones.
# It fetches nothing and deletes nothing.
#
# Usage: bench/merged-branches.sh [REMOTE] [MAIN]      (default: origin main)
#
# A branch counts as merged when its head is an ancestor of REMOTE/MAIN. A branch that was
# squashed or rebased into main is not an ancestor and is listed as not merged. In a shallow or
# single branch clone a head may be missing locally, or the history may end before the two
# meet: those are listed as unknown, never as merged.
set -u
REMOTE=${1:-origin}
MAIN=${2:-main}
cd "$(dirname "$0")/.." || exit 2

heads=$(git ls-remote --heads "$REMOTE") || { echo "cannot reach remote $REMOTE" >&2; exit 2; }
main_sha=$(printf '%s\n' "$heads" | awk -v r="refs/heads/$MAIN" '$2 == r { print $1 }')
[ -n "$main_sha" ] || { echo "$REMOTE has no branch $MAIN" >&2; exit 2; }
if ! git cat-file -e "$main_sha^{commit}" 2>/dev/null; then
    echo "the head of $REMOTE/$MAIN ($main_sha) is not in this clone: git fetch $REMOTE $MAIN" >&2
    exit 2
fi
shallow=$(git rev-parse --is-shallow-repository 2>/dev/null || echo false)

merged=""
n_merged=0 n_open=0 n_unknown=0
# no pipe into the loop: the counters must survive it
tmp=$(mktemp)
trap 'rm -f "$tmp"' EXIT
printf '%s\n' "$heads" >"$tmp"
while read -r sha ref; do
    name=${ref#refs/heads/}
    [ "$name" = "$MAIN" ] && continue
    if ! git cat-file -e "$sha^{commit}" 2>/dev/null; then
        echo "unknown (commit not fetched)   $name"
        n_unknown=$((n_unknown + 1))
    elif git merge-base --is-ancestor "$sha" "$main_sha" 2>/dev/null; then
        echo "merged                         $name"
        merged="$merged $name"
        n_merged=$((n_merged + 1))
    elif [ "$shallow" = true ] && ! git merge-base "$sha" "$main_sha" >/dev/null 2>&1; then
        echo "unknown (history too shallow)  $name"
        n_unknown=$((n_unknown + 1))
    else
        echo "not merged                     $name"
        n_open=$((n_open + 1))
    fi
done <"$tmp"

echo
echo "$n_merged merged into $REMOTE/$MAIN, $n_open not merged, $n_unknown unknown"
if [ -n "$merged" ]; then
    echo
    echo "To delete the merged branches on $REMOTE (this script does not run it):"
    echo "  git push $REMOTE --delete$merged"
fi
if [ "$n_unknown" -gt 0 ]; then
    echo
    echo "Unknown branches need their commits first, e.g. git fetch $REMOTE <branch>, or a full clone."
fi
