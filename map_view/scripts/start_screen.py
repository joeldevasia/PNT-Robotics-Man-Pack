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
from std_msgs.msg import Float32, Int32, Float32MultiArray
from PyQt5.QtGui import QIcon, QPixmap, QTransform
from sensor_msgs.msg import NavSatFix
from PyQt5 import QtGui
import wmctrl
from PyQt5.QtGui import QWindow
import resources

screen = None

# creating a class
# that inherits the QDialog class
class Window(QMainWindow):

	# constructor
	def __init__(self):
		QMainWindow.__init__(self)
		self.rospkg_path = rospkg.RosPack()
		self.map_view_package_path = self.rospkg_path.get_path('map_view')

		uic.loadUi(self.map_view_package_path+"/scripts/ManPack_UI.ui", self)

		# setting window title
		self.setWindowTitle("Insert Configuration Parameter")
		height = screen.size().height()*1.0
		width = (height*1200)/900
		self.setFixedSize(width, height)
		
		self.nda_bot_package_path = self.rospkg_path.get_path('nda_bot')
		self.initial_coordinates_file_path = self.map_view_package_path+"/config/initial_coordinates.yaml"
		self.config_file_path = self.nda_bot_package_path+"/config/config.yaml"

		self.magnetic_direction = rospy.Subscriber('/magnetic_dir_degrees', Float32, self.magnetic_dir_callback)
		self.waypoint_direction = rospy.Subscriber('/waypoint_dir_degrees', Float32, self.waypoint_dir_callback)
		self.distance_covered_sub = rospy.Subscriber('/distance', Float32, self.distance_covered_callback)
		self.displacement_sub = rospy.Subscriber('/displacement', Float32, self.displacement_callback)
		self.speed_sub = rospy.Subscriber('/speed', Float32, self.speed_callback)
		self.lat_long_sub = rospy.Subscriber('/navsat/fix', NavSatFix, self.lat_long_callback)
		self.easting_northing_sub = rospy.Subscriber('/easting_northing', Float32MultiArray, self.easting_northing_callback)
		self.totoal_steps_sub = rospy.Subscriber('/steps', Int32, self.total_steps_callback)																

		self.magnetometer_direction_pixmap = QPixmap(self.map_view_package_path+"/assets/red-direction.png")
		self.waypoint_direction_pixmap = QPixmap(self.map_view_package_path+"/assets/white-direction.png")

		uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
		roslaunch.configure_logging(uuid)
		self.launch = roslaunch.parent.ROSLaunchParent(uuid, [self.map_view_package_path+"/launch/map_view.launch"])

		self.stackedWidget = self.findChild(QStackedWidget, 'stackedWidget')
		self.stackedWidget.setCurrentIndex(0)

		self.User_ID_Line_Edit = self.findChild(QLineEdit, 'User_ID_Line_Edit')
		self.Password_Line_Edit = self.findChild(QLineEdit, 'Password_Line_Edit')
		self.Initializing_text = self.findChild(QLabel, 'Initializing_text')
		self.Initializing_text.setText("")
		self.Enter_Button = self.findChild(QPushButton, 'Enter_Button')

		self.Enter_Button.clicked.connect(self.authenticate_user)

		self.Latitude_Line_Edit = self.findChild(QLineEdit, 'Latitude_Line_Edit')
		self.Longitude_Line_Edit = self.findChild(QLineEdit, 'Longitude_Line_Edit')
		self.Easting_Line_Edit = self.findChild(QLineEdit, 'Easting_Line_Edit')
		self.Northing_Line_Edit = self.findChild(QLineEdit, 'Northing_Line_Edit')
		self.Zone_Line_Edit = self.findChild(QLineEdit, 'Zone_Line_Edit')
		self.Stride_Length_Line_Edit = self.findChild(QLineEdit, 'Stride_Length_Line_Edit')
		self.Load_Button = self.findChild(QPushButton, 'Load_Button')
		self.Save_Button = self.findChild(QPushButton, 'Save_Button')
		self.Cancel_Button = self.findChild(QPushButton, 'Cancel_Button')

		self.Load_Button.clicked.connect(self.load_parameters)
		self.Save_Button.clicked.connect(self.save_parameters)
		self.Cancel_Button.clicked.connect(self.close)

		self.Latitude_Line_Edit.textChanged.connect(self.clear_easting_northing)
		self.Longitude_Line_Edit.textChanged.connect(self.clear_easting_northing)

		self.Easting_Line_Edit.textChanged.connect(self.clear_latitude_longitude)
		self.Northing_Line_Edit.textChanged.connect(self.clear_latitude_longitude)
		self.Zone_Line_Edit.textChanged.connect(self.clear_latitude_longitude)

		self.Current_Direction_Label = self.findChild(QLabel, 'Current_Direction_Label')
		self.Waypoint_Direction_Label = self.findChild(QLabel, 'Waypoint_Direction_Label')

		self.Speed_Label = self.findChild(QLabel, 'Speed_Label')
		self.Distance_Label = self.findChild(QLabel, 'Distance_Label')
		self.Displacement_Label = self.findChild(QLabel, 'Displacement_Label')
		self.Magnetic_Bearing_Label = self.findChild(QLabel, 'Magnetic_Bearing_Label')
		self.Latitude_Label = self.findChild(QLabel, 'Latitude_Label')
		self.Longitude_Label = self.findChild(QLabel, 'Longitude_Label')
		self.Easting_Label = self.findChild(QLabel, 'Easting_Label')
		self.Northing_Label = self.findChild(QLabel, 'Northing_Label')

		self.Mapviz_Layout = self.findChild(QVBoxLayout, 'Mapviz_Layout')

		self.Total_Steps_Label = self.findChild(QLabel, 'Total_Steps_Label')

		self.scale_widgets()

	def scale_widgets(self):
        # Get screen size
		screen_size = QApplication.primaryScreen().availableGeometry()

        # Design-time size
		design_width = 1200
		design_height = 900

		window_height = screen.size().height()*0.9
		window_width = (window_height*1200)/900
		self.setFixedSize(window_width, window_height)

		scale_factor = window_height/design_height  # Maintain aspect ratio

        # Scale all widgets
		for widget in self.findChildren(QWidget):
            # Scale geometry
			geom = widget.geometry()
			widget_height = geom.height()*scale_factor
			widget_width = geom.width()*scale_factor
			
			geom.setWidth(int(widget_width))
			geom.setHeight(int(widget_height))
			geom.moveTo(int(geom.x() * scale_factor), int(geom.y() * scale_factor))

			widget.setGeometry(geom)

			# Scale font
			font = widget.font()
			font.setPointSizeF(font.pointSizeF() * scale_factor)
			widget.setFont(font)

	def speed_callback(self, msg):
		self.Speed_Label.setText(str(msg.data))

	def distance_covered_callback(self, msg):
		self.Distance_Label.setText(str(msg.data))

	def displacement_callback(self, msg):
		self.Displacement_Label.setText(str(msg.data))

	def lat_long_callback(self, msg):
		self.Latitude_Label.setText(str(round(msg.latitude,5)))
		self.Longitude_Label.setText(str(round(msg.longitude,5)))
	
	def easting_northing_callback(self, msg):
		self.Easting_Label.setText(str(msg.data[0]))
		self.Northing_Label.setText(str(msg.data[1]))

	def total_steps_callback(self, msg):
		self.Total_Steps_Label.setText(str(msg.data))
	

	def magnetic_dir_callback(self, msg):
		try: 
			rotated_pixmap = self.magnetometer_direction_pixmap.transformed(QTransform().rotate(360.0-msg.data))
			self.Current_Direction_Label.setPixmap(rotated_pixmap)
			self.Magnetic_Bearing_Label.setText(str(int(360-msg.data)))
		except Exception as e:
			print(e)

	def waypoint_dir_callback(self, msg):
		try: 
			rotated_pixmap = self.waypoint_direction_pixmap.transformed(QTransform().rotate(360.0-msg.data))
			self.Waypoint_Direction_Label.setPixmap(rotated_pixmap)
		except Exception as e:
			print(e)


	def clear_easting_northing(self):
		self.Easting_Line_Edit.clear()
		self.Northing_Line_Edit.clear()
		self.Zone_Line_Edit.clear()

	def clear_latitude_longitude(self):
		self.Latitude_Line_Edit.clear()
		self.Longitude_Line_Edit.clear()

	def authenticate_user(self):
		if str(self.User_ID_Line_Edit.text()) == "army" and str(self.Password_Line_Edit.text()) == "1234" :
			for i in range(0,2):
				self.Initializing_text.setText("Initializing")
				self.Initializing_text.repaint()
				rospy.sleep(0.2)
				self.Initializing_text.setText("Initializing.")
				self.Initializing_text.repaint()
				rospy.sleep(0.2)
				self.Initializing_text.setText("Initializing..")
				self.Initializing_text.repaint()
				rospy.sleep(0.2)
				self.Initializing_text.setText("Initializing...")
				self.Initializing_text.repaint()
				rospy.sleep(0.2)
			self.stackedWidget.setCurrentIndex(1)

		
	def load_parameters(self):
        # Load and parse the map.yaml file
		# print(os.getcwd()+'/src/carpack_map_view/config/map.yaml')
		with open(self.initial_coordinates_file_path, 'r') as file:
			yaml_data = yaml.safe_load(file)
			latitude = yaml_data['local_xy_origins'][0]['latitude']
			longitude = yaml_data['local_xy_origins'][0]['longitude']
			self.Latitude_Line_Edit.setText(str(latitude))
			self.Longitude_Line_Edit.setText(str(longitude))
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

		if self.Latitude_Line_Edit.text() == "" and self.Longitude_Line_Edit.text() == "":

			try:
				e = float(self.Easting_Line_Edit.text())
				n = float(self.Northing_Line_Edit.text())
				z = int(self.Zone_Line_Edit.text())
				s = float(self.Stride_Length_Line_Edit.text())
			except Exception as exception:
				print(exception)
				return
			if not (42 <=z <=47):
				print("Zone Should be between 42 and 47")
				return
			
			latitude_longitude = utm.to_latlon(easting=float(self.Easting_Line_Edit.text()), northing=float(self.Northing_Line_Edit.text()), zone_number=int(self.Zone_Line_Edit.text()), northern=True)
			latitude = latitude_longitude[0]
			longitude = latitude_longitude[1]

			with open(self.initial_coordinates_file_path, 'w') as file:
				coordinates_yaml_data['local_xy_origins'][0]['latitude'] = float(latitude)
				coordinates_yaml_data['local_xy_origins'][0]['longitude'] = float(longitude)
				yaml.dump(coordinates_yaml_data, file)

		elif self.Easting_Line_Edit.text() == "" and self.Northing_Line_Edit.text() == "":

			try:
				f = float(self.Latitude_Line_Edit.text())
				f = float(self.Longitude_Line_Edit.text())
				i = float(self.Stride_Length_Line_Edit.text())
			except Exception as e:
				print(e)
				return
				
			
			latitude = float(self.Latitude_Line_Edit.text())
			longitude = float(self.Longitude_Line_Edit.text())

			with open(self.initial_coordinates_file_path, 'w') as file:
				coordinates_yaml_data['local_xy_origins'][0]['latitude'] = float(latitude)
				coordinates_yaml_data['local_xy_origins'][0]['longitude'] = float(longitude)
				yaml.dump(coordinates_yaml_data, file)

		with open(self.config_file_path, 'w') as file:
			config_yaml_data['configuration'][0]['stride_length'] = float(self.Stride_Length_Line_Edit.text())
			yaml.dump(config_yaml_data, file)

		
		# self.close()
		self.stackedWidget.setCurrentIndex(2)
		# self.roslaunch.start()

		# rospy.sleep(10.0)
		self.launch.start()
		rospy.loginfo("started")
		rospy.sleep(6.0)
		
		mapviz_id = None
		
		while mapviz_id is None:
			# Get the list of windows
			list = wmctrl.Window.list()
			#Find mapviz window
			for window in list:
				if window.wm_class == 'mapviz.mapviz':
					mapviz_id = int(window.id, 16)
					break

		mapviz_window = QWindow.fromWinId(mapviz_id)
		mapviz_window.show()

		# mapviz_window.show()
		mapviz_window.setFlags(Qt.FramelessWindowHint)

		mapviz_window.hide()
		rospy.sleep(1.0)

		mapviz_widget = QWidget.createWindowContainer(mapviz_window, self)
		self.Mapviz_Layout.addWidget(mapviz_widget)
		

		# rospy.spin()
		# launch.shutdown()

	# def roslaunch_shutdown(self):
	# 	try:
	# 		self.launch.shutdown()
	# 	except Exception as e:
	# 		print(e)

	def __del__(self):
		# try:
			self.launch.shutdown()
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

