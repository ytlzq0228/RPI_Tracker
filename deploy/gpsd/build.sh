#!/bin/sh
# Build only the daemon; do not replace Debian's gpsd or client libraries.
# Usage: sh build.sh /path/to/gpsd-release-3.22.tar.gz /path/to/workdir
set -eu
archive=$(readlink -f "$1")
workdir=$2
patchdir=$(CDPATH= cd -- "$(dirname -- "$0")" && pwd)
expected=13a6097d94a4209ff144094af7fe74afbd375f0722316666326372b0cde96c30
actual=$(sha256sum "$archive" | cut -d ' ' -f 1)
[ "$actual" = "$expected" ] || { echo 'Unexpected source archive checksum' >&2; exit 1; }
mkdir -p "$workdir"
tar -xzf "$archive" -C "$workdir"
cd "$workdir/gpsd-release-3.22"
patch -p1 < "$patchdir/0001-accept-rtcm3x.patch"
scons -j2 gpsd-3.22/gpsd/gpsd shared=no manbuild=no python=no qt=no
# Source: https://github.com/ntpsec/gpsd/tree/release-3.22
# Install the resulting daemon separately under /opt/gpsd-3.22-rtcm3x/sbin/.
# shared=no keeps the existing distro libgps ABI and packages untouched.
