#!/usr/bin/env bash
# git-sync.sh — one direction of the two-way GitHub <-> Forgejo sync for this repo.
#
# Runs in CI on the host that just changed (SRC) and pushes that change to the other
# host (DST). It is FAST-FORWARD ONLY: it never force-pushes, so if the same branch
# has diverged on both hosts the push is rejected and the job fails loudly. Nothing is
# ever overwritten; a human merges the two versions (see "Fixing a red run" below).
#
# The same script runs on both hosts; only the environment differs:
#   REPO_DIR   path of a *bare* clone of SRC, made by the workflow step before this one
#   DST_URL    https push URL of the other host with a username but NO token,
#              e.g. https://x@github.com/ChainLynxRobotics/Romi-Projects.git
#   DST_TOKEN  token for DST (handed to git via GIT_ASKPASS; never put in a URL or log)
#   EVENT      github.event_name: push | delete | workflow_dispatch
#   REF        github.ref for push events (refs/heads/<branch> or refs/tags/<tag>)
#   DEL_REF    github.event.ref for delete events (short name, e.g. my-branch)
#   DEL_TYPE   github.event.ref_type for delete events (branch | tag)
#   ON_DELETE  what to do on DST when a branch is deleted on SRC:
#                safe   (default) delete it on DST only if DST's copy has no commits that
#                       SRC no longer has anywhere; otherwise fail and leave it alone
#                always delete it on DST regardless
#                never  leave DST alone
#
# What a push event syncs:
#   * the ref that was just pushed (created on DST if it is new there), plus
#   * every other branch/tag that already exists on BOTH hosts and differs — so a run
#     that was missed (runner down, token expired) is caught up by the next push.
#   Branches that exist only on DST are left alone, so a branch deleted on SRC is not
#   resurrected by unrelated pushes coming back from DST.
# workflow_dispatch pushes every branch and tag (union of refs) — a manual catch-up.
#
# Why no loop: the push to DST triggers DST's copy of this workflow, which clones DST,
# finds every ref already identical on SRC, and exits with "Nothing to sync".
#
# Fixing a red run ("! [rejected] <branch> -> <branch> (fetch first)"):
#   git clone https://git.emeraldcityrobotics.org/chainlynx/Romi-Projects.git && cd Romi-Projects
#   git remote add github https://github.com/ChainLynxRobotics/Romi-Projects.git && git fetch github
#   git checkout <branch> && git merge github/<branch>     # resolve conflicts, commit
#   git push origin <branch>                               # the sync carries it on to GitHub
#
set -euo pipefail

: "${REPO_DIR:?}" "${DST_URL:?}" "${DST_TOKEN:?}" "${EVENT:?}"
ON_DELETE="${ON_DELETE:-safe}"

# Hand the token to git through an askpass helper so it never appears in a URL or in logs.
tmp="$(mktemp -d)"
trap 'rm -rf "$tmp"' EXIT
printf '#!/bin/sh\nprintf %%s "$DST_TOKEN"\n' > "$tmp/askpass"
chmod +x "$tmp/askpass"
export GIT_ASKPASS="$tmp/askpass" GIT_TERMINAL_PROMPT=0 DST_TOKEN

cd "$REPO_DIR"

# Refs currently on the other host, as "<ref>\t<sha>" (peeled "^{}" tag lines dropped).
git ls-remote --heads --tags "$DST_URL" | awk '$2 !~ /\^\{\}$/ { print $2 "\t" $1 }' > "$tmp/dst"
dst_sha() { awk -v r="$1" -F'\t' '$1 == r { print $2 }' "$tmp/dst"; }

# ---------------------------------------------------------------- delete events
if [ "$EVENT" = "delete" ]; then
  case "${DEL_TYPE:-}" in
    branch) ref="refs/heads/${DEL_REF:?}" ;;
    tag)    ref="refs/tags/${DEL_REF:?}" ;;
    *) echo "::error::unknown ref_type '${DEL_TYPE:-}'"; exit 1 ;;
  esac
  sha="$(dst_sha "$ref")"
  if [ -z "$sha" ]; then
    echo "$ref is already gone on the other host."; exit 0
  fi
  if [ "$ON_DELETE" = "never" ]; then
    echo "ON_DELETE=never: leaving $ref on the other host."; exit 0
  fi
  # A bare clone only contains objects reachable from SRC's remaining refs, so if the
  # other host's tip is present here, deleting it there loses nothing.
  if [ "$ON_DELETE" = "safe" ] && [ "$DEL_TYPE" = "branch" ] && ! git cat-file -e "${sha}^{commit}" 2>/dev/null; then
    echo "::error::$ref was deleted here, but the other host's copy ($sha) has commits this host no longer has (e.g. it was squash-merged). Not deleting it — delete it there by hand if that was intended."
    exit 1
  fi
  git push "$DST_URL" ":$ref"
  echo "Deleted $ref on the other host."
  exit 0
fi

# ---------------------------------------------------------------- push / dispatch
want=()
while IFS=' ' read -r sha ref; do
  case "$ref" in refs/heads/*|refs/tags/*) ;; *) continue ;; esac
  there="$(dst_sha "$ref")"
  [ "$there" = "$sha" ] && continue                          # identical on both hosts
  if [ "$EVENT" = "workflow_dispatch" ] || [ -n "$there" ] || [ "$ref" = "${REF:-}" ]; then
    want+=("$ref:$ref")
  fi
done < <(git for-each-ref --format='%(objectname) %(refname)' refs/heads refs/tags)

if [ "${#want[@]}" -eq 0 ]; then
  echo "Nothing to sync — the other host already has everything."
  exit 0
fi

echo "Pushing (fast-forward only): ${want[*]}"
if ! git push "$DST_URL" "${want[@]}"; then
  echo "::error::Push rejected — at least one branch has diverged between GitHub and Forgejo. Nothing was overwritten. Merge the two versions by hand (see the header of scripts/git-sync.sh) and push again."
  exit 1
fi
echo "Done."
