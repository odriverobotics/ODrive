#!/bin/bash
set -euo pipefail

if [ $# -eq 1 ]; then
    OUTPUT="$1"
else
    OUTPUT="/dev/stdout"
fi

# The git root lies outside of the tup root
export GIT_DISCOVERY_ACROSS_FILESYSTEM=1

# Get a description of the current Git state
# Examples of what this string may become:
#   fw-v0.3.6              The current commit is exactly at tag "fw-v0.3.6"
#                          There may or may not be untracked files in the
#                          working directory.
#   fw-v0.3.6*             The current commit is at tag "fw-v0.3.6" and there
#                          are uncommitted changes in the working directory.
#   fw-v0.3.6-4-g3703ae5   The working directory at a commit with hash 3703ae5,
#                          4 commits ahead of tag fw-v0.3.6 and clean.
FW_VERSION="$(git describe --always --tags --dirty=* || echo "[unknown commit]")"

# Extract version numbers
FW_VERSION_MAJOR="$(sed -n 's/.*v\([0-9a-zA-Z]\).\([0-9a-zA-Z]\).\([0-9a-zA-Z]\)\(.*\)/\1/p' <<< "$FW_VERSION")"
FW_VERSION_MINOR="$(sed -n 's/.*v\([0-9a-zA-Z]\).\([0-9a-zA-Z]\).\([0-9a-zA-Z]\)\(.*\)/\2/p' <<< "$FW_VERSION")"
FW_VERSION_REVISION="$(sed -n 's/.*v\([0-9a-zA-Z]\).\([0-9a-zA-Z]\).\([0-9a-zA-Z]\)\(.*\)/\3/p' <<< "$FW_VERSION")"
FW_VERSION_SUFFIX="$(sed -n 's/.*v\([0-9a-zA-Z]\).\([0-9a-zA-Z]\).\([0-9a-zA-Z]\)\(.*\)/\4/p' <<< "$FW_VERSION")"

# Fall back to 0 if the verions does not match the expected pattern
[ "$FW_VERSION_MAJOR" == "" ] && FW_VERSION_MAJOR=0
[ "$FW_VERSION_MINOR" == "" ] && FW_VERSION_MINOR=0
[ "$FW_VERSION_REVISION" == "" ] && FW_VERSION_REVISION=0

if [ "$FW_VERSION_SUFFIX" == "" ]; then
    FW_VERSION_UNRELEASED=0
else
    FW_VERSION_UNRELEASED=1
fi

cat > "$OUTPUT" <<EOF
#define FW_VERSION "$FW_VERSION"
#define FW_VERSION_MAJOR $FW_VERSION_MAJOR
#define FW_VERSION_MINOR $FW_VERSION_MINOR
#define FW_VERSION_REVISION $FW_VERSION_REVISION
#define FW_VERSION_UNRELEASED $FW_VERSION_UNRELEASED
EOF
