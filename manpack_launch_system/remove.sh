#!/bin/sh

LOGFILE=latest-uninstall.log
rm latest-uninstall.log
echo "#####################################################################" | tee -a $LOGFILE
echo "#####################################################################" | tee -a $LOGFILE

echo "Deleting Manpack docker image" | tee -a $LOGFILE
sudo docker rmi pnt/manpack:latest | tee -a $LOGFILE

sudo echo "Deleting MapServer docker image" | tee -a $LOGFILE
sudo docker rmi klokantech/openmaptiles-server:latest | tee -a $LOGFILE

echo "Removing Docker..." | tee -a $LOGFILE

# sudo systemctl stop docker

sudo apt-get purge docker.io bridge-utils containerd runc pigz ubuntu-fan wmctrl -y | tee -a $LOGFILE

sudo systemctl unmask docker.service | tee -a $LOGFILE

sudo systemctl unmask docker.socket | tee -a $LOGFILE

echo "Docker removed successfully!" | tee -a $LOGFILE

echo "#########################" | tee -a $LOGFILE
echo "Removing docker group" | tee -a $LOGFILE

sudo groupdel docker | tee -a $LOGFILE

echo "Removed docker group successfully!" | tee -a $LOGFILE

echo "#########################" | tee -a $LOGFILE

echo "Removing ManPack Launch System and Dependencies..." | tee -a $LOGFILE

sudo apt remove manpack-launch-system -y | tee -a $LOGFILE
sudo apt remove wmctrl -y | tee -a $LOGFILE
pip uninstall attrs wmctrl inflection qstylizer tinycss2 webencodings -y | tee -a $LOGFILE

echo "ManPack Launch System and Dependencies removed successfully!" | tee -a $LOGFILE
echo  "System Will Reboot now in 5 seconds" | tee -a $LOGFILE
echo "#####################################################################" | tee -a $LOGFILE
echo "#####################################################################" | tee -a $LOGFILE
sleep 5

reboot

