#!/bin/bash
# Builds Surfer with surfer_wheel_scroll.patch: the mouse wheel over the waves
# scrolls signals up and down instead of panning time.
#
#   support/build_surfer.sh <work_dir>          build and install to ~/.local/bin
#   NO_INSTALL=1 support/build_surfer.sh <dir>  build only
#
# The Rust toolchain and the source stay inside <work_dir>, so nothing else on the
# system changes and deleting <work_dir> afterwards removes them. It needs a few GB.
set -euo pipefail

VERSION=v0.7.0
HERE=$(cd "$(dirname "$0")" && pwd)
WORK=${1:?usage: build_surfer.sh <work_dir>}
mkdir -p "$WORK"
WORK=$(cd "$WORK" && pwd)

export RUSTUP_HOME="$WORK/rustup"
export CARGO_HOME="$WORK/cargo"
export PATH="$CARGO_HOME/bin:$PATH"

if [ ! -x "$CARGO_HOME/bin/cargo" ]; then
  curl -fsSL --proto '=https' --tlsv1.2 -o "$WORK/rustup-init" \
    https://static.rust-lang.org/rustup/dist/x86_64-unknown-linux-gnu/rustup-init
  chmod +x "$WORK/rustup-init"
  "$WORK/rustup-init" -y --no-modify-path --profile minimal --default-toolchain stable
fi
cargo --version

SRC="$WORK/surfer"
if [ ! -d "$SRC" ]; then
  git clone --quiet --depth 1 --branch "$VERSION" --recurse-submodules --shallow-submodules \
    https://gitlab.com/surfer-project/surfer.git "$SRC"
fi
cd "$SRC"
# Apply once; on a re-run the patch is already in. A patch that does not fit stops here.
if ! git apply --reverse --check "$HERE/surfer_wheel_scroll.patch" 2>/dev/null; then
  git apply "$HERE/surfer_wheel_scroll.patch"
fi
git diff --stat

cargo build --release --locked --bin surfer --jobs "${JOBS:-8}"
BIN="$SRC/target/release/surfer"
"$BIN" --version
echo "built: $BIN"

if [ -z "${NO_INSTALL:-}" ]; then
  mkdir -p "$HOME/.local/bin"
  if [ -f "$HOME/.local/bin/surfer" ] && [ ! -f "$HOME/.local/bin/surfer.official" ]; then
    cp "$HOME/.local/bin/surfer" "$HOME/.local/bin/surfer.official"
  fi
  install -m 755 "$BIN" "$HOME/.local/bin/surfer"
  echo "installed: $HOME/.local/bin/surfer"
fi
