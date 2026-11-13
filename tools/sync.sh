#!/usr/bin/env bash
# Keeps the home PC and the work PC on the same RTL, TB and sim setup through
# GitHub. Normally run as `make sync` (or `make pull`) from sim/.
#
#   tools/sync.sh ["commit message"]   commit this PC's changes, rebase them on
#                                      the other PC's, push
#   tools/sync.sh --pull-only          only bring in the other PC's changes
set -euo pipefail
cd "$(git rev-parse --show-toplevel)"

# sim/out logs are marked merge=keeplocal in .gitattributes: when both PCs ran
# the same configuration, keep this PC's log instead of stopping on a conflict.
# During a rebase %B is this PC's commit being re-applied.
git config merge.keeplocal.name "keep this PC's simulation log"
git config merge.keeplocal.driver 'cp -f %B %A'

# Both PCs work on main and every sync pushes to main.
branch=main
if [ "$(git rev-parse --abbrev-ref HEAD)" != "$branch" ]; then
  echo "You are on '$(git rev-parse --abbrev-ref HEAD)', not $branch. Run 'git switch $branch' first."
  exit 1
fi
case "$(uname -s)" in MINGW*|MSYS*|CYGWIN*) pc=windows ;; *) pc=linux ;; esac

pull() {
  git fetch -q origin
  if git rev-parse -q --verify "refs/remotes/origin/$branch" >/dev/null; then
    before=$(git rev-parse HEAD)
    if ! git rebase -q --autostash "origin/$branch"; then
      echo
      echo "CONFLICT: both PCs changed the same lines. Fix the files git lists above,"
      echo "then run 'git add <file>' and 'git rebase --continue' (or 'git rebase --abort')."
      exit 1
    fi
    if [ "$before" != "$(git rev-parse HEAD)" ]; then
      echo "pulled from the other PC:"
      git --no-pager diff --stat "$before" HEAD -- RTL TB sim | tail -20
    fi
  fi
}

if [ "${1:-}" = "--pull-only" ]; then
  pull
  echo "up to date: $(git log --oneline -1)"
  exit 0
fi

msg=${1:-"sync $pc "}
git add -A
if git diff --cached --quiet; then
  echo "nothing new to commit on this PC"
else
  git --no-pager diff --cached --stat | tail -20
  git commit -q -m "$msg"
  echo "committed: $msg"
fi
pull
git push -q -u origin "$branch"
echo "in sync with origin/$branch: $(git log --oneline -1)"
