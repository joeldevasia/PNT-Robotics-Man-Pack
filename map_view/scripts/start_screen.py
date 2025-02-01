#!/usr/bin/env python3
from PyQt5.QtWidgets import *
from PyQt5.QtCore import * 
import sys
import yaml
import os
import rospy
import rospkg
import roslaunch
from PyQt5 import uic
import utm
import threading
from std_msgs.msg import Float32, Int32, Float32MultiArray, String
from PyQt5.QtGui import QIcon, QPixmap, QTransform
from sensor_msgs.msg import NavSatFix
from PyQt5 import QtGui
import wmctrl
from PyQt5.QtGui import QWindow
import qstylizer.parser
import resources
import time

screen = None

# creating a class
# that inherits the QDialog class
class Window(QMainWindow):

	# constructor
	def __init__(self):
		QMainWindow.__init__(self)
		self.rospkg_path = rospkg.RosPack()
		self.map_view_package_path = self.rospkg_path.get_path('map_view')
		self.view_sensor_package_path = self.rospkg_path.get_path('view_sensor')

		uic.loadUi(self.map_view_package_path+"/scripts/ManPack_Latest.ui", self)

		# setting window title
		self.setWindowTitle("Insert Configuration Parameter")
		height = round(screen.size().height()*0.9)
		width = round((height*1300)/740)
		self.showMaximized()
		# self.setFixedSize(width, height)

		self.start_time = None
		self.elapsed_time = None
		
		self.nda_bot_package_path = self.rospkg_path.get_path('nda_bot')
		self.initial_coordinates_file_path = self.map_view_package_path+"/config/initial_coordinates.yaml"
		self.config_file_path = self.nda_bot_package_path+"/config/config.yaml"

		self.magnetic_direction = rospy.Subscriber('/sensors/magnetometer', Int32, self.magnetic_direction_callback)
		self.distance_covered_sub = rospy.Subscriber('/odom/distance', String, self.distance_covered_callback)
		self.speed_sub = rospy.Subscriber('/odom/speed', String, self.speed_callback)
		self.lat_long_sub = rospy.Subscriber('/navsat/fix', NavSatFix, self.lat_long_callback)

		# self.magnetometer_direction_pixmap = QPixmap(self.map_view_package_path+"/assets/red-direction.png")
		# self.waypoint_direction_pixmap = QPixmap(self.map_view_package_path+"/assets/white-direction.png")

		uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
		roslaunch.configure_logging(uuid)
		self.mapviz_launch = roslaunch.parent.ROSLaunchParent(uuid, [self.map_view_package_path+"/launch/map_view.launch"])
		self.rviz_launch = roslaunch.parent.ROSLaunchParent(uuid, [self.view_sensor_package_path+"/launch/sensors.launch"])

		self.stackedWidget = self.findChild(QStackedWidget, 'stackedWidget')
		self.stackedWidget.setCurrentIndex(0)

		self.Enter_Username_LineEdit = self.findChild(QLineEdit, 'Enter_Username_LineEdit')
		self.Enter_Password_LineEdit = self.findChild(QLineEdit, 'Enter_Password_LineEdit')
		self.Login_PushButton = self.findChild(QPushButton, 'Login_PushButton')

		self.Login_PushButton.clicked.connect(self.authenticate_user)

		self.Enter_Latitude_LineEdit = self.findChild(QLineEdit, 'Enter_Latitude_LineEdit')
		self.Enter_Longitude_LineEdit = self.findChild(QLineEdit, 'Enter_Longitude_LineEdit')
		self.Load_PushButton = self.findChild(QPushButton, 'Load_PushButton')
		self.Save_PushButton = self.findChild(QPushButton, 'Save_PushButton')
		self.Start_PushButton = self.findChild(QPushButton, 'Start_PushButton')

		self.Load_PushButton.clicked.connect(self.load_parameters)
		self.Save_PushButton.clicked.connect(self.save_parameters)
		self.Start_PushButton.clicked.connect(self.start_system)

		self.Speed_Label = self.findChild(QLabel, 'Current_Speed_Label')
		self.Distance_Label = self.findChild(QLabel, 'Distance_Covered_Label')
		self.Current_Direction_Label = self.findChild(QLabel, 'Current_Direction_Label')
		self.Current_Latitude_Label = self.findChild(QLabel, 'Current_Latitude_Label')
		self.Current_Longitude_Label = self.findChild(QLabel, 'Current_Longitude_Label')
		self.Elapsed_Time_Label = self.findChild(QLabel, 'Elapsed_Time_Label')
		self.Waypoints_Reached_Label = self.findChild(QLabel, 'Waypoints_Reached_Label')
		self.Total_Distance_Label = self.findChild(QLabel, 'Total_Distance_Label')
		self.ETA_Label = self.findChild(QLabel, 'ETA_Label')
		print("#################") 
		widgets = self.findChild(QVBoxLayout,"MapViz_Layout")
		print(widgets.objectName())
		self.MapViz_Layout = self.findChild(QVBoxLayout,"MapViz_Layout")

		self.View_Sensor_data_PushButton = self.findChild(QPushButton, 'View_Sensor_data_PushButton')
		self.Sensors_Page_Back_PushButton = self.findChild(QPushButton, 'Sensors_Page_Back_PushButton')

		self.View_Sensor_data_PushButton.clicked.connect(self.view_sensor_page)
		self.Sensors_Page_Back_PushButton.clicked.connect(self.back_to_map_page)


		self.RViz_Layout = self.findChild(QVBoxLayout, 'RViz_Layout')

		self.Enter_Username_LineEdit.setText("army")
		self.Enter_Password_LineEdit.setText("1234")

		self.scale_widgets()

	def scale_widgets(self):
        # Get screen size
		screen_size = QApplication.primaryScreen().availableGeometry()

        # Design-time size
		design_width = 1300
		design_height = 740

		window_height = round(screen.size().height()*0.9)
		window_width = round((window_height*1300)/740)
		# self.setFixedSize(window_width, window_height)

		scale_factor = window_height/design_height  # Maintain aspect ratio

        # Scale all widgets
		for widget in self.findChildren(QWidget):
            # Scale geometry
			geom = widget.geometry()
			widget_height = geom.height()*scale_factor
			widget_width = geom.width()*scale_factor
			
			geom.setWidth(round(widget_width))
			geom.setHeight(round(widget_height))
			geom.moveTo(round(geom.x() * scale_factor), round(geom.y() * scale_factor))

			widget.setGeometry(geom)

			# Scale font
			font = widget.font()
			font.setPointSizeF(font.pointSizeF() * scale_factor)
			widget.setFont(font)
 
	def magnetic_direction_callback(self, msg):
		self.Current_Direction_Label.setText(str(msg.data))

	def speed_callback(self, msg):
		self.Speed_Label.setText(str(msg.data))

	def distance_covered_callback(self, msg):
		self.Distance_Label.setText(str(msg.data))

	def lat_long_callback(self, msg):
		self.Current_Latitude_Label.setText(str(round(msg.latitude,5)))
		self.Current_Longitude_Label.setText(str(round(msg.longitude,5)))

	def authenticate_user(self):
		if str(self.Enter_Username_LineEdit.text()) == "army" and str(self.Enter_Password_LineEdit.text()) == "1234" :
			self.stackedWidget.setCurrentIndex(1)

		
	def load_parameters(self):
        # Load and parse the map.yaml file
		# print(os.getcwd()+'/src/carpack_map_view/config/map.yaml')
		with open(self.initial_coordinates_file_path, 'r') as file:
			yaml_data = yaml.safe_load(file)
			latitude = yaml_data['local_xy_origins'][0]['latitude']
			longitude = yaml_data['local_xy_origins'][0]['longitude']
			self.Enter_Latitude_LineEdit.setText(str(latitude))
			self.Enter_Longitude_LineEdit.setText(str(longitude))
			file.close()
		
		with open(self.config_file_path, 'r') as file:
			yaml_data = yaml.safe_load(file)
			stride_length = yaml_data['configuration'][0]['stride_length']
			self.Stride_Length_Line_Edit.setText(str(stride_length))
			file.close()
			
	def save_parameters(self):
        # Load and parse the map.yaml file
		with open(self.initial_coordinates_file_path, 'r') as file:
			coordinates_yaml_data = yaml.safe_load(file)

		with open(self.config_file_path, 'r') as file:
			config_yaml_data = yaml.safe_load(file)

		latitude = 0.0
		longitude = 0.0

		if self.Enter_Latitude_LineEdit.text() != "" and self.Enter_Longitude_LineEdit.text() != "":

			try:
				f = float(self.Enter_Latitude_LineEdit.text())
				f = float(self.Enter_Longitude_LineEdit.text())
			except Exception as e:
				print(e)
				return
				
			
			latitude = float(self.Enter_Latitude_LineEdit.text())
			longitude = float(self.Enter_Longitude_LineEdit.text())

			with open(self.initial_coordinates_file_path, 'w') as file:
				coordinates_yaml_data['local_xy_origins'][0]['latitude'] = float(latitude)
				coordinates_yaml_data['local_xy_origins'][0]['longitude'] = float(longitude)
				yaml.dump(coordinates_yaml_data, file)

	def start_system(self):
		self.stackedWidget.setCurrentIndex(2)
		# self.roslaunch.start()

		# rospy.sleep(10.0)
		self.mapviz_launch.start()
		self.rviz_launch.start()
		rospy.loginfo("started")
		rospy.sleep(6.0)
		
		mapviz_id = None
		rviz_id = None
		
		while mapviz_id is None:
			# Get the list of windows
			list = wmctrl.Window.list()
			#Find mapviz window
			for window in list:
				if window.wm_class == 'mapviz.mapviz':
					mapviz_id = int(window.id, 16)
					break

		while rviz_id is None:
			# Get the list of windows
			list = wmctrl.Window.list()
			#Find rviz window
			for window in list:
				if window.wm_class == 'rviz.rviz':
					rviz_id = int(window.id, 16)
					break

		mapviz_window = QWindow.fromWinId(mapviz_id)
		mapviz_window.show()

		rviz_window = QWindow.fromWinId(rviz_id)
		rviz_window.show()

		# mapviz_window.show()
		mapviz_window.setFlags(Qt.FramelessWindowHint)
		# rviz_window.show()
		rviz_window.setFlags(Qt.FramelessWindowHint)

		mapviz_window.hide()
		rviz_window.hide()
		rospy.sleep(1.0)

		mapviz_widget = QWidget.createWindowContainer(mapviz_window, self)
		rviz_widget = QWidget.createWindowContainer(rviz_window, self)
		self.MapViz_Layout.addWidget(mapviz_widget)
		self.RViz_Layout.addWidget(rviz_widget)

		self.start_time = rospy.Time.now()
		self.elapsed_time = rospy.Time.now()
		self.time_timer = rospy.Timer(rospy.Duration(1), self.update_time)

	def view_sensor_page(self):
		self.stackedWidget.setCurrentIndex(3)

	def back_to_map_page(self):
		self.stackedWidget.setCurrentIndex(2)
	
	def update_time(self, timer):
		self.elapsed_time = rospy.Time.now() - self.start_time
		formated_time = time.strftime('%H:%M:%S', time.gmtime(self.elapsed_time.secs))
		self.Elapsed_Time_Label.setText(str(formated_time))

	def __del__(self):
		# try:
			self.mapviz_launch.shutdown()
			self.rviz_launch.shutdown()
		# except Exception as e:
		# 	print(e)

# main method
if __name__ == '__main__':
	rospy.init_node('initial_parameters_gui', anonymous=True)

	# create pyqt5 app
	app = QApplication(sys.argv)
	app.setWindowIcon(QtGui.QIcon(':/Assets/assets/logo.png'))
	screen = app.primaryScreen()

	# create the instance of our Window
	window = Window()

	# showing the window
	window.show()

	# rospy.signal_shutdown("System Exited")

	# start the app
	sys.exit(app.exec())

