#!/bin/bash
# Check if running as root
if [ "$EUID" -ne 0 ]; then
  echo "Please run as root"
  exit
fi
# Check if correct number of arguments provided
if [ "$#" -ne 2 ]; then
  echo "Usage: $0 <architecture> <distro>"
  exit
fi
ARCHITECTURE=$1
DISTRO=$2
# Current directory
CURRENT_DIR=$(pwd)
# URL of the HTML page with the list of Docker files
HTML_URL="https://download.docker.com/linux/ubuntu/dists/${DISTRO}/pool/stable/${ARCHITECTURE}/"
# Fetch the list of available packages
PACKAGE_LIST=$(curl -s "$HTML_URL" | grep -oP '(?<=href=")[^"]+(?=")' | grep -oP '.*\.deb' | sort -Vr)
# Function to download a package with a progress bar
download_package() {
  PACKAGE=$1
  FILENAME="${PACKAGE}"
  URL="${HTML_URL}${FILENAME}"
  echo "Downloading $FILENAME..."
  curl -L --progress-bar "$URL" -o "$CURRENT_DIR/$FILENAME"
}
# Download the latest package for each type
# TYPES=("containerd.io" "docker-ce" "docker-ce-cli" "docker-buildx-plugin" "docker-compose-plugin")
TYPES=("docker.io")
for TYPE in "${TYPES[@]}"; do
  PACKAGE=$(echo "$PACKAGE_LIST" | grep "$TYPE" | head -n 1)
  if [ -n "$PACKAGE" ]; then
    download_package "$PACKAGE"
  fi
done
echo "Docker packages downloaded successfully!"