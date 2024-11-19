#!/bin/sh

sudo groupadd docker
echo "#########################"
echo "Adding user to docker group"
sudo usermod -aG docker $USER
# newgrp docker
# su - $USER
echo "User added to docker group successfully!"
echo "#########################"

echo  "Installing Docker and Dependencies..."
# sudo apt-get update 
# sudo apt-get install docker.io -y
sudo apt install ./installation_packages/*.deb -y
echo "Docker installed successfully!"
echo "#########################"

echo  "Restarting docker service..."
sudo systemctl restart docker
echo "Docker service restarted successfully!"
echo "#########################"

echo  "Loading ManPack Docker image"
sudo docker image load --input docker_images/manpack_docker_image_1.tar.gz
sudo docker images
echo "Docker image loaded successfully!"
echo "#########################"

echo "Installing ManPack Launch System"
sudo dpkg -i ./package/manpack-launch-system.deb
echo "ManPack Launch System installed successfully!"
echo "#########################"

echo  "System Will Reboot now in 5 seconds"
sleep 5
reboot