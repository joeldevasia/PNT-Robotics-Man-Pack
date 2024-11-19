#!/usr/bin/env python3
from PyQt5.QtWidgets import *
from PyQt5.QtCore import *
import sys
import os
import rospy
from PyQt5 import uic
from PyQt5 import QtGui
import wmctrl
from PyQt5.QtGui import QWindow
import resources
import subprocess
# from ManPack_Host_UI import Ui_MainWindow

screen = None


# creating a class
# that inherits the QDialog class
class Window(QMainWindow):

    # constructor		
	def __init__(self):
		QMainWindow.__init__(self)

        # window = QWidget()
        # window.show()  # IMPORTANT!!!!! Windows are hidden by default.
		
		# uic.loadUi(":/UI/ManPack_Host_UI.ui", self)
		# self.setupUi(self)

		window_height = screen.size().height()*0.9
		window_width = (window_height*1200)/900
		self.setFixedSize(window_width, window_height)

		# self.Mapviz_Layout = self.findChild(QVBoxLayout, 'verticalLayout')

        # docker run -it --rm -e DISPLAY --network host --privileged -v /tmp/.X11-unix:/tmp/.X11-unix -v /etc/localtime:/etc/localtime:ro --name manpack_container pnt/manpack:latest
		subprocess.call(["xhost", "+local:"])
		subprocess.call(["systemctl","restart","docker"])
		self.docker_process = subprocess.Popen(["docker", "run", "-it", "--rm", "-e", "DISPLAY", "--network", "host", "--privileged", "-v", "/tmp/.X11-unix:/tmp/.X11-unix", "-v", "/etc/localtime:/etc/localtime:ro", "--name", "manpack_container", "pnt/manpack:latest"])

		mapviz_id = None
		
		while mapviz_id is None:
			# Get the list of windows
			list = wmctrl.Window.list()
			#Find mapviz window
			for window in list:
				if window.wm_class == 'start_screen.py.start_screen.py':
					mapviz_id = int(window.id, 16)
					break

		mapviz_window = QWindow.fromWinId(mapviz_id)
		mapviz_window.show()

		# mapviz_window.show()
		mapviz_window.setFlags(Qt.FramelessWindowHint)

		mapviz_window.hide()
		rospy.sleep(1.0)

		mapviz_widget = QWidget.createWindowContainer(mapviz_window, self)
		self.setCentralWidget(mapviz_widget)

		# self.Mapviz_Layout.addWidget(mapviz_widget)

	def __del__(self):
		# self.docker_process.terminate()
		self.docker_process.kill()
		subprocess.Popen(["docker", "kill", "manpack_container"])


# main method
if __name__ == "__main__":
	app = QApplication(sys.argv)
	app.setWindowIcon(QtGui.QIcon(":/Assets/logo.png"))
	screen = app.primaryScreen()
	window = Window()
	window.show()
	app.exec()
	subprocess.Popen(["docker", "kill", "manpack_container"])
	sys.exit()
