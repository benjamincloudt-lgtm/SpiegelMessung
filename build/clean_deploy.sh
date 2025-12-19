#!/bin/bash
set -e

APP="SpiegelMessung.app"
EXECUTABLE="$APP/Contents/MacOS/SpiegelMessung"
FRAMEWORKS="$APP/Contents/Frameworks"

echo "==> Preparing App Bundle..."
# Qt-Standard-Deployment (kopiert Plugins & Frameworks)
/opt/homebrew/opt/qt@5/bin/macdeployqt "$APP"

echo "==> Bundling OpenCV and other dependencies..."
# dylibbundler kopiert alles Fehlende und fixiert Pfade auf @executable_path
brew install dylibbundler 2>/dev/null || true
dylibbundler -od -b -x "$EXECUTABLE" -d "$FRAMEWORKS/" -p "@executable_path/../Frameworks/"

echo "==> Removing system paths (RPATHs)..."
# Entfernt harte Pfade zu Homebrew, damit nur interne Libs genutzt werden
for rpath in $(otool -l "$EXECUTABLE" | grep LC_RPATH -A2 | grep path | awk '{print $2}'); do
    install_name_tool -delete_rpath "$rpath" "$EXECUTABLE" 2>/dev/null || true
done

echo "==> Adding internal RPATH..."
install_name_tool -add_rpath "@executable_path/../Frameworks" "$EXECUTABLE"

echo "==> Cleaning metadata and signing..."
# Entfernt detritus/xattrs und signiert alles neu
xattr -cr "$APP"
codesign --force --deep --sign - "$APP"

echo "✅ DONE! You can now test the app."
