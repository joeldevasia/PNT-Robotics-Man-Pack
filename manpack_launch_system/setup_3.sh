#!/bin/sh

# echo "Installing Docker..."

# sudo apt-get update 
# sudo apt-get install docker.io -y

# sudo systemctl start docker

# echo "Docker installed successfully!"

# echo "Restarting docker service..."

# sudo systemctl restart docker

# echo "User added to docker group successfully!"

# echo "#########################"

echo "####### Loading Docker image"

sudo docker image load --input docker_images/manpack_docker_image_1.tar.gz

echo "Docker image loaded successfully!"

# echo "#########################"
# echo "Adding user to docker group"
# sudo groupadd docker
# sudo usermod -aG docker $USER
# newgrp docker
# echo "User added to docker group successfully!"

sudo docker images

echo "#########################"

echo "System Will Start in 3 seconds"
sleep 5

reboot
