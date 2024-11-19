#!/bin/sh
# Create folders.
[ -e package ] && rm -r package
mkdir -p package/opt/manpack-launch-system
mkdir -p package/usr/share/applications
mkdir -p package/usr/share/icons/hicolor/scalable/apps

# Copy files (change icon names, add lines for non-scaled icons)
cp -r dist/manpack-launch-system package/opt/manpack-launch-system/manpack-launch-system
cp icons/logo.svg package/usr/share/icons/hicolor/scalable/apps/manpack-launch-system.svg
cp manpack-launch-system.desktop package/usr/share/applications

# Change permissions
find package/opt/manpack-launch-system -type f -exec chmod 644 -- {} +
find package/opt/manpack-launch-system -type d -exec chmod 755 -- {} +
find package/usr/share -type f -exec chmod 644 -- {} +
chmod +x package/opt/manpack-launch-system/manpack-launch-system