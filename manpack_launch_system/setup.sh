#!/bin/sh

echo "Installing Docker..."

sudo apt-get update 
sudo apt-get install docker.io -y

sudo systemctl start docker

echo "Docker installed successfully!"

echo "#########################"
echo "Adding user to docker group"
sudo groupadd docker
sudo usermod -aG docker $USER
newgrp docker

sudo systemctl restart docker

echo "User added to docker group successfully!"

echo "#########################"

docker image load --input manpack_docker_image.tar.gz