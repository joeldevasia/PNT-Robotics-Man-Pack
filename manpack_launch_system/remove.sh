#!/bin/sh

echo "Deleting Manpack docker image"
docker rmi pnt/manpack:latest

echo "Removing Docker..."

# sudo systemctl stop docker

sudo apt-get remove docker.io -y

sudo apt-get autoremove -y

sudo systemctl unmask docker.service

sudo systemctl unmask docker.socket

echo "Docker removed successfully!"

echo "#########################"
echo "Removing docker group"

sudo groupdel docker

echo "Removed docker group successfully!"

echo "#########################"

reboot

