#!/bin/sh
# SPDX-License-Identifier: GPL-3.0-only
# Builds Ductile and installs it with its desktop entry, icon and file type.
#   packaging/linux/install.sh            # into ~/.local
#   PREFIX=/usr/local sudo -E packaging/linux/install.sh
set -eu
here=$(cd "$(dirname "$0")" && pwd)
root=$(cd "$here/../.." && pwd)
prefix=${PREFIX:-$HOME/.local}
id=io.github.fcoltro.Ductile

if [ ! -x "$root/target/release/ductile" ] || [ "${REBUILD:-1}" = 1 ]; then
    (cd "$root" && cargo build --release -p ductile-app)
fi
install -Dm755 "$root/target/release/ductile" "$prefix/bin/ductile"
install -Dm644 "$here/$id.desktop" "$prefix/share/applications/$id.desktop"
install -Dm644 "$here/$id.metainfo.xml" "$prefix/share/metainfo/$id.metainfo.xml"
install -Dm644 "$here/ductile.svg" "$prefix/share/icons/hicolor/scalable/apps/$id.svg"
install -Dm644 "$here/ductile.png" "$prefix/share/icons/hicolor/256x256/apps/$id.png"
install -Dm644 "$here/ductile-mime.xml" "$prefix/share/mime/packages/$id.xml"
command -v update-mime-database >/dev/null && update-mime-database "$prefix/share/mime" || true
command -v update-desktop-database >/dev/null && update-desktop-database "$prefix/share/applications" || true
command -v gtk-update-icon-cache >/dev/null && gtk-update-icon-cache -qt "$prefix/share/icons/hicolor" || true
echo "Installed Ductile into $prefix"
