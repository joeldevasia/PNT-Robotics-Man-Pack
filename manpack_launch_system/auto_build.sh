#!/bin/sh

echo "########################"
echo "Building ManPack System Executable"

pyinstaller --onefile --noconsole --clean manpack-launch-system.py

echo "ManPack System Executable Built Successfully!"
echo "########################"

echo "Generating Debian Package"

sh ./package.sh

fpm -C package -s dir -t deb -n "manpack-launch-system" -v 0.1.0 -p installation_packages/manpack-launch-system.deb

#fpm

echo "########################"
echo "Debian Package Generated Successfully!"