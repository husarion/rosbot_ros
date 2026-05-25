# Release driver. Day-to-day build/test stays at the colcon workspace level (CLAUDE.md §3).

[private]
default:
    @just --list

venv := env_var_or_default("ROSBOT_ROS_RELEASE_VENV", "$HOME/.venv-rosbot-ros-release")

install-deps:
    #!/bin/bash
    set -euo pipefail
    if [[ ! -d {{venv}} ]]; then
      python3 -m venv {{venv}}
    fi
    {{venv}}/bin/pip install --upgrade pip
    {{venv}}/bin/pip install pre-commit
    echo "Release tooling installed in {{venv}}."

# Cut a release for the current branch: pre-commit gate → claude drafts
# version + CHANGELOG section → apply-release.py bumps package.xml + prepends
# CHANGELOG → y/N → commit + tag + push (CI publishes via .github/workflows/release.yaml).
# Tags: bare X.Y.Z on jazzy, X.Y.Z-<branch-suffix> elsewhere.
release:
    #!/bin/bash
    set -euo pipefail
    if [[ ! -x {{venv}}/bin/pre-commit ]]; then
        echo "release: 'pre-commit' not in {{venv}} — bootstrapping..."
        just install-deps
    fi
    export PATH="{{venv}}/bin:$PATH"

    # ---- 1. sanity ----
    [ -z "$(git status --porcelain)" ] \
        || { echo "release: working tree dirty — commit or stash first" >&2; exit 1; }
    branch=$(git branch --show-current)
    [ "$branch" != "main" ] \
        || { echo "release: must not be on 'main' (jazzy is the canonical branch)" >&2; exit 1; }
    if git remote get-url origin >/dev/null 2>&1 \
       && git fetch --quiet origin "$branch" 2>/dev/null \
       && git rev-parse --verify "origin/$branch" >/dev/null 2>&1; then
        [ "$(git rev-parse HEAD)" = "$(git rev-parse "origin/$branch")" ] \
            || { echo "release: local '$branch' is not in sync with origin/$branch (push or rebase first)" >&2; exit 1; }
    else
        echo "release: warning — couldn't compare against origin/$branch (skipping sync check)"
    fi
    command -v claude >/dev/null \
        || { echo "release: 'claude' CLI not on PATH (https://docs.claude.com/en/docs/claude-code)" >&2; exit 1; }
    command -v jq >/dev/null \
        || { echo "release: 'jq' not on PATH" >&2; exit 1; }
    command -v pre-commit >/dev/null \
        || { echo "release: 'pre-commit' still missing after install-deps" >&2; exit 1; }

    # ---- 2. tag suffix + commit range ----
    if [ "$branch" = "jazzy" ]; then
        tag_suffix=""
    else
        tag_suffix="-$(echo "$branch" | awk -F'/' '{print $NF}')"
    fi
    last_tag=$(git tag --list "[0-9]*${tag_suffix}" --sort=-v:refname | head -n1 || true)
    if [ -n "$last_tag" ]; then
        range="${last_tag}..HEAD"
        range_desc="since ${last_tag}"
    else
        last_any=$(git tag --list "[0-9]*" --sort=-v:refname | head -n1 || true)
        if [ -n "$last_any" ]; then
            range="${last_any}..HEAD"
            range_desc="since ${last_any} (first release on the ${tag_suffix#-} track)"
        else
            range=""
            range_desc="full history (first release ever)"
        fi
    fi
    commits=$(git log ${range:+$range} --no-merges --pretty='%h %s')
    [ -n "$commits" ] || { echo "release: no commits ${range_desc} — nothing to release." >&2; exit 1; }
    current_version=$(sed -nE 's,.*<version>([^<]+)</version>.*,\1,p' rosbot/package.xml | head -n1)
    [ -n "$current_version" ] || { echo "release: couldn't read <version> from rosbot/package.xml" >&2; exit 1; }

    echo "=== ${branch} ${range_desc} (current rosbot/package.xml version: ${current_version}) ==="
    printf '%s\n' "$commits" | sed 's/^/  /'
    echo

    # ---- 3. local gate: pre-commit ----
    echo "=== local gate: pre-commit run -a ==="
    pre-commit run -a
    echo "gate ok"
    echo

    # ---- 4. headless claude → version + changelog section ----
    echo "=== asking claude for version + changelog section ==="
    prompt=$(mktemp); out=$(mktemp); section=$(mktemp)
    trap 'rm -f "$prompt" "$out" "$section"' EXIT
    {
        printf 'You are preparing a release of rosbot_ros (ROS 2 jazzy driver\n'
        printf 'and bringup stack for Husarion ROSbot 2 / 3 / XL).\n\n'
        printf 'Current rosbot/package.xml version: %s\n' "$current_version"
        printf 'Tag this release will get: X.Y.Z%s\n\n' "$tag_suffix"
        printf 'Commits to describe (newest first), %s:\n\n' "$range_desc"
        printf '%s\n\n' "$commits"
        printf 'Use the Read tool on CHANGELOG.md to match the existing tone\n'
        printf 'and section style. If CHANGELOG.md has no prior entries yet,\n'
        printf 'still follow Keep-a-Changelog conventions strictly.\n\n'
        printf 'Format: Keep-a-Changelog. Sections inside the entry are\n'
        printf '### Added / Changed / Fixed / Removed (omit empty groups).\n'
        printf 'Bullets must be concise and user-facing. Audience: a robotics\n'
        printf 'engineer reading the release page to decide whether to upgrade.\n'
        printf 'Group related commits; skip pure repo housekeeping (lint,\n'
        printf 'CLAUDE.md tweaks) unless substantial. Reference packages by\n'
        printf 'their colcon names (rosbot_bringup, rosbot_controller, etc.)\n'
        printf 'when a change is package-scoped. Call out interaction with the\n'
        printf 'rosbot-firmware release lineage when relevant (especially for\n'
        printf 'MAVLink-track entries).\n\n'
        printf 'Decide the next semver bump for X.Y.Z:\n'
        printf '  patch -> bug fixes, dependency bumps, doc-only churn\n'
        printf '  minor -> new user-facing features, backwards-compatible launch args\n'
        printf '  major -> breaking changes / removals (topic renames, launch arg\n'
        printf '           removals, ROS API changes documented in ROS_API.md)\n\n'
        printf 'Do NOT include the ## header — the recipe prepends it with today\\047s date.\n\n'
        printf 'Your final message MUST be exactly one JSON object, no prose, no\n'
        printf 'code fence, on a single line:\n\n'
        printf '  {"version":"X.Y.Z","section":"### Added\\\\n- foo\\\\n\\\\n### Fixed\\\\n- bar"}\n\n'
        printf 'Newlines inside the section field JSON-escaped as \\\\n.\n'
    } > "$prompt"

    claude -p "$(cat "$prompt")" --allowed-tools Read --output-format json > "$out"
    raw=$(jq -r '.result // empty' "$out")
    [ -n "$raw" ] || { echo "release: claude returned empty result" >&2; cat "$out" >&2; exit 1; }
    raw=$(printf '%s' "$raw" | sed -e '/^```/d')
    version=$(printf '%s' "$raw" | jq -r '.version')
    section_body=$(printf '%s' "$raw" | jq -r '.section')
    [[ "$version" =~ ^[0-9]+\.[0-9]+\.[0-9]+$ ]] \
        || { echo "release: invalid version '$version' from claude" >&2; printf '%s\n' "$raw" >&2; exit 1; }
    [ -n "$section_body" ] || { echo "release: empty section from claude" >&2; exit 1; }

    new_tag="${version}${tag_suffix}"
    echo "claude proposes: ${current_version} -> ${new_tag}"
    echo

    # ---- 5. apply locally ----
    printf '%s\n' "$section_body" > "$section"
    python3 .release/apply-release.py "$new_tag" "$section"

    # ---- 6. y/N gate, commit + tag + push ----
    echo
    echo "=== diff for the release commit ==="
    git --no-pager diff --stat
    echo
    git --no-pager diff CHANGELOG.md
    echo
    read -rp "Commit, tag ${new_tag}, and push to origin? [y/N] " confirm
    if [ "${confirm:-N}" != "y" ] && [ "${confirm:-N}" != "Y" ]; then
        echo "aborted — reverting working-tree edits"
        git restore --worktree CHANGELOG.md $(git ls-files '**/package.xml')
        find . -name '*.bak' -path '*/package.xml.bak' -delete
        rm -f CHANGELOG.md.bak
        exit 1
    fi

    find . -name '*.bak' -path '*/package.xml.bak' -delete
    rm -f CHANGELOG.md.bak
    git add CHANGELOG.md $(git ls-files '**/package.xml')
    git commit -m "Release ${new_tag}"
    git tag -a "${new_tag}" -m "Release ${new_tag}"
    git push --follow-tags origin "${branch}"
    echo
    echo "=== ${new_tag} released — CI is now building artefacts ==="
    if command -v gh >/dev/null 2>&1; then
        repo_url=$(gh repo view --json url -q .url 2>/dev/null || true)
        [ -n "$repo_url" ] && echo "  ${repo_url}/actions"
        [ -n "$repo_url" ] && echo "  ${repo_url}/releases/tag/${new_tag}"
    fi
