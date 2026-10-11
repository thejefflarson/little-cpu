#!/bin/bash
# Installs NIST's SP800-90B_EntropyAssessment at one pinned commit: fetches the commit's tarball,
# refuses it unless the SHA-256 matches, builds ea_non_iid and ea_restart from it and stamps the pin.
set -euo pipefail
if [ "$#" -ne 4 ]; then
  echo "usage: nist90b_setup.sh <commit> <sha256> <dest> <download-dir>" >&2
  exit 2
fi
COMMIT=$1; SHA256=$2; DEST=$3; DOWNLOADS=$4
STAMP="$COMMIT $SHA256"

if [ -x "$DEST/bin/ea_non_iid" ] && [ -x "$DEST/bin/ea_restart" ] && [ "$(cat "$DEST/.nist90b-pin" 2>/dev/null)" = "$STAMP" ]; then
  echo "SP800-90B_EntropyAssessment $COMMIT already built in $DEST"
  exit 0
fi

if command -v shasum >/dev/null 2>&1; then sha='shasum -a 256'
elif command -v sha256sum >/dev/null 2>&1; then sha='sha256sum'
else echo "neither shasum nor sha256sum is on PATH; refusing to unpack a tarball this machine cannot check." >&2; exit 1; fi

mkdir -p "$DOWNLOADS"
tgz=$DOWNLOADS/SP800-90B_EntropyAssessment-$COMMIT.tar.gz
if [ ! -f "$tgz" ]; then
  url=https://codeload.github.com/usnistgov/SP800-90B_EntropyAssessment/tar.gz/$COMMIT
  echo "fetching $url"
  curl -fsSL -o "$tgz.part" "$url"
  mv "$tgz.part" "$tgz"
fi
got=$($sha "$tgz" | cut -d ' ' -f 1)
if [ "$got" != "$SHA256" ]; then
  echo "SP800-90B_EntropyAssessment tarball SHA-256 MISMATCH -- refusing to extract:" >&2
  echo "  expected : $SHA256" >&2
  echo "  actual   : $got" >&2
  rm -f "$tgz"
  exit 1
fi

top=SP800-90B_EntropyAssessment-$COMMIT/
tar tzf "$tgz" | awk -v top="$top" '
  index($0, top) != 1 { print "member outside " top ": " $0 > "/dev/stderr"; bad = 1 }
  /(^|\/)\.\.(\/|$)/  { print "traversal in member: " $0 > "/dev/stderr"; bad = 1 }
  END { exit bad ? 1 : 0 }' || { echo "refusing to extract the tarball" >&2; exit 1; }

mkdir -p "$(dirname "$DEST")"
tmp=$(mktemp -d "$DEST.XXXXXX")
trap 'rm -rf "$tmp"' EXIT
tar xzf "$tgz" -C "$tmp" --strip-components=1 --no-same-owner --no-same-permissions

# Upstream's Makefile assumes g++, GNU OpenMP and /usr/include; Apple's clang has neither a -fopenmp
# nor jsoncpp's headers, so macOS builds with clang++, Homebrew's libomp and Homebrew's prefixes.
if [ "$(uname -s)" = Darwin ]; then
  command -v brew >/dev/null || { echo "Homebrew is needed for libomp, jsoncpp, libdivsufsort and openssl@3" >&2; exit 1; }
  for formula in libomp jsoncpp libdivsufsort openssl@3; do
    brew --prefix "$formula" >/dev/null 2>&1 || { echo "missing: brew install $formula" >&2; exit 1; }
  done
  omp=$(brew --prefix libomp); json=$(brew --prefix jsoncpp); dss=$(brew --prefix libdivsufsort); ssl=$(brew --prefix openssl@3)
  arch=x86; [ "$(uname -m)" = arm64 ] && arch=arm
  make -C "$tmp/cpp" non_iid restart ARCH=$arch CXX=clang++ \
    CXXFLAGS="-std=c++11 -Xpreprocessor -fopenmp -O2 -ffloat-store -I$json/include -I$dss/include -I$ssl/include -I$omp/include" \
    LIB="-L$dss/lib -L$omp/lib -lomp -lbz2 -lpthread -ldivsufsort -ldivsufsort64" \
    SHARED_LIB="-L$json/lib -L$ssl/lib -ljsoncpp -lcrypto" > "$tmp/build.log" 2>&1 || { tail -30 "$tmp/build.log"; exit 1; }
else
  make -C "$tmp/cpp" non_iid restart > "$tmp/build.log" 2>&1 || { tail -30 "$tmp/build.log"; echo "needs libbz2, libdivsufsort, libjsoncpp, libssl dev packages" >&2; exit 1; }
fi

mkdir -p "$tmp/bin"
cp "$tmp/cpp/ea_non_iid" "$tmp/cpp/ea_restart" "$tmp/bin/"
printf '%s' "$STAMP" > "$tmp/.nist90b-pin"
rm -rf "$DEST"
mv "$tmp" "$DEST"
trap - EXIT
"$DEST/bin/ea_non_iid" --version | head -3 || true
