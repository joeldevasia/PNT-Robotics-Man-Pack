#!/bin/sh

LOGFILE=latest-install.log
rm latest-install.log
echo "#####################################################################"| tee -a $LOGFILE
echo "#####################################################################"| tee -a $LOGFILE
sudo groupadd docker
echo "#########################"| tee -a $LOGFILE
echo "Adding user to docker group"| tee -a $LOGFILE
sudo usermod -aG docker $USER| tee -a $LOGFILE
# newgrp docker
# su - $USER
echo "User added to docker group successfully!"| tee -a $LOGFILE
echo "#########################"| tee -a $LOGFILE

echo  "Installing Docker and Dependencies..."| tee -a $LOGFILE
# sudo apt-get update 
# sudo apt-get install docker.io -y
# sudo apt install ./installation_packages/*.deb -y
sudo apt install ./installation_packages/bridge-utils.deb ./installation_packages/containerd.deb ./installation_packages/docker.io.deb ./installation_packages/runc.deb ./installation_packages/pigz.deb ./installation_packages/ubuntu-fan.deb ./installation_packages/wmctrl.deb -y| tee -a $LOGFILE

echo "Docker and Dependencies installed successfully!"| tee -a $LOGFILE
echo "#########################"| tee -a $LOGFILE

echo  "Restarting docker service..."| tee -a $LOGFILE
sudo systemctl restart docker| tee -a $LOGFILE
echo "Docker service restarted successfully!"| tee -a $LOGFILE
echo "#########################"| tee -a $LOGFILE

echo  "Loading ManPack Docker image"| tee -a $LOGFILE
sudo docker image load --input docker_images/manpack_docker_image_1.tar.gz| tee -a $LOGFILE

echo  "Loading MapServer Docker image"| tee -a $LOGFILE
sudo docker image load --input docker_images/mapserver_docker_image.tar.gz| tee -a $LOGFILE

sudo docker images| tee -a $LOGFILE
echo "Docker images loaded successfully!"| tee -a $LOGFILE
echo "#########################"| tee -a $LOGFILE

echo "Installing ManPack Launch System and Dependencies..."| tee -a $LOGFILE
sudo dpkg -i ./installation_packages/manpack-launch-system.deb| tee -a $LOGFILE
pip install ./installation_packages/attrs-24.2.0-py3-none-any.whl ./installation_packages/wmctrl-0.5-py2.py3-none-any.whl ./installation_packages/inflection-0.5.1-py2.py3-none-any.whl ./installation_packages/qstylizer-0.2.4-py2.py3-none-any.whl ./installation_packages/tinycss2-1.4.0-py3-none-any.whl ./installation_packages/webencodings-0.5.1-py2.py3-none-any.whl| tee -a $LOGFILE

echo "ManPack Launch System and Dependencies installed successfully!"| tee -a $LOGFILE
echo "#########################"| tee -a $LOGFILE

echo  "System Will Reboot now in 5 seconds"| tee -a $LOGFILE
echo "#####################################################################"| tee -a $LOGFILE
echo "#####################################################################"| tee -a $LOGFILE
sleep 5
reboot
