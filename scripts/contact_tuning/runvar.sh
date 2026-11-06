#!/bin/bash
# usage: runvar.sh <variants file> <model> <states> <outdir> [parallel]
source "$(dirname "$0")/env.sh"
cd "$(dirname "$0")"
mkdir -p "$4"
cat "$1" | xargs -P ${5:-12} -L 1 bash -c 'tag=$0; $PY replay.py '"$2 $3 $4"'/$tag.json tag=$tag "$@" > '"$4"'/$tag.log 2>&1'
