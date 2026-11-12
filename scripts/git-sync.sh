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
#   * every other branch that exists on BOTH hosts where SRC is strictly ahead — so a run
#     that was missed (runner down, token expired) is caught up by the next push.
#   Branches where DST is ahead are reported and left alone (DST's own run brings them
#   here). Branches that exist only on DST are left alone, so a branch deleted on SRC is
#   not resurrected by unrelated pushes coming back from DST.
# workflow_dispatch also pushes every branch/tag that is missing on DST — a manual catch-up.
#
# Why no loop: the push to DST triggers DST's copy of this workflow, which clones DST,
# finds every ref already identical on SRC, and exits with "Nothing to sync".
#
# CAVEAT (both GitHub and Forgejo): a push only triggers the workflows that exist on the
# pushed branch. So a push to a branch that does not contain this script and the workflow
# files (e.g. an old feature branch) runs nothing; it is carried over by the next push to
# main on that host (catch-up above). To make such a branch sync on its own pushes, merge
# main into it once. Branches created from main after this landed sync on every push.
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
# Fetch the other host's branch tips (into a private namespace, no tags) so we can tell
# "we are ahead" (push), "they are ahead" (they will push to us) and "diverged" (fail) apart.
git fetch --quiet --no-tags "$DST_URL" '+refs/heads/*:refs/dst/heads/*' 2>/dev/null || true

want=(); behind=(); diverged=()
while IFS=' ' read -r sha ref; do
  case "$ref" in refs/heads/*|refs/tags/*) ;; *) continue ;; esac
  there="$(dst_sha "$ref")"
  [ "$there" = "$sha" ] && continue                          # identical on both hosts
  if [ -z "$there" ]; then                                   # not on the other host yet
    if [ "$EVENT" = "workflow_dispatch" ] || [ "$ref" = "${REF:-}" ]; then want+=("$ref:$ref"); fi
    continue
  fi
  case "$ref" in
    refs/tags/*) diverged+=("$ref (tag points at a different commit on each host)") ;;
    *) if git merge-base --is-ancestor "$there" "$sha" 2>/dev/null; then
         want+=("$ref:$ref")                                 # fast-forward: we are ahead
       elif git merge-base --is-ancestor "$sha" "$there" 2>/dev/null; then
         behind+=("$ref")                                    # they are ahead: nothing to do here
       else
         diverged+=("$ref")
       fi ;;
  esac
done < <(git for-each-ref --format='%(objectname) %(refname)' refs/heads refs/tags)

[ "${#behind[@]}" -gt 0 ] && echo "Other host is ahead on: ${behind[*]} (its own sync run brings those here)."

if [ "${#want[@]}" -gt 0 ]; then
  echo "Pushing (fast-forward only): ${want[*]}"
  if ! git push "$DST_URL" "${want[@]}"; then
    echo "::error::Push rejected (a push landed on the other host at the same moment?). Nothing was overwritten; the next sync run will retry."
    exit 1
  fi
elif [ "${#diverged[@]}" -eq 0 ]; then
  echo "Nothing to sync — the other host already has everything."
fi

if [ "${#diverged[@]}" -gt 0 ]; then
  echo "::error::DIVERGED between GitHub and Forgejo: ${diverged[*]} — commits exist on both hosts that the other lacks. Nothing was overwritten. Merge the two versions by hand (see the header of scripts/git-sync.sh) and push the merge to either host."
  exit 1
fi
echo "Done."
