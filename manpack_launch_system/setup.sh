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
cd installation_packages
sudo apt install ./bridge-utils.deb ./containerd.deb ./docker.io.deb ./runc.deb ./pigz.deb ./ubuntu-fan.deb ./wmctrl.deb -y| tee -a $LOGFILE
cd ..
echo "Docker and Dependencies installed successfully!"| tee -a $LOGFILE
echo "#########################"| tee -a $LOGFILE

echo  "Restarting docker service..."| tee -a $LOGFILE
sudo systemctl restart docker| tee -a $LOGFILE
echo "Docker service restarted successfully!"| tee -a $LOGFILE
echo "#########################"| tee -a $LOGFILE

echo  "Loading ManPack Docker image"| tee -a $LOGFILE
sudo docker image load --input docker_images/manpack_docker_image_1.tar.gz| tee -a $LOGFILE
sudo docker images| tee -a $LOGFILE
echo "Docker image loaded successfully!"| tee -a $LOGFILE
echo "#########################"| tee -a $LOGFILE

echo "Installing ManPack Launch System and Dependencies..."| tee -a $LOGFILE
cd installation_packages
sudo dpkg -i ./manpack-launch-system.deb| tee -a $LOGFILE
pip install ./attrs-24.2.0-py3-none-any.whl ./wmctrl-0.5-py2.py3-none-any.whl ./inflection-0.5.1-py2.py3-none-any.whl ./qstylizer-0.2.4-py2.py3-none-any.whl ./tinycss2-1.4.0-py3-none-any.whl ./webencodings-0.5.1-py2.py3-none-any.whl| tee -a $LOGFILE
cd ..
echo "ManPack Launch System and Dependencies installed successfully!"| tee -a $LOGFILE
echo "#########################"| tee -a $LOGFILE

echo  "System Will Reboot now in 5 seconds"| tee -a $LOGFILE
echo "#####################################################################"| tee -a $LOGFILE
echo "#####################################################################"| tee -a $LOGFILE
sleep 5
reboot
