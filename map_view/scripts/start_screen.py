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
from PyQt5.QtCore import QProcess, QEvent

screen = None

# creating a class
# that inherits the QDialog class
class Window(QMainWindow):

	# constructor
	def __init__(self):
		QMainWindow.__init__(self)

		self.setFocusPolicy(Qt.ClickFocus)

		self.rospkg_path = rospkg.RosPack()
		self.map_view_package_path = self.rospkg_path.get_path('map_view')

		uic.loadUi(self.map_view_package_path+"/scripts/ManPack_UI.ui", self)

		# setting window title
		self.setWindowTitle("Insert Configuration Parameter")
		height = round(screen.size().height()*1.0)
		width = round((height*1200)/900)
		self.setFixedSize(width, height)
		
		self.nda_bot_package_path = self.rospkg_path.get_path('nda_bot')
		self.initial_coordinates_file_path = self.map_view_package_path+"/config/initial_coordinates.yaml"
		self.config_file_path = self.nda_bot_package_path+"/config/config.yaml"

		self.magnetic_direction = rospy.Subscriber('/sensors/magnetometer', Int32, self.magnetic_dir_callback)
		self.waypoint_direction = rospy.Subscriber('/waypoint_dir_degrees', Float32, self.waypoint_dir_callback)
		self.distance_covered_sub = rospy.Subscriber('/odom/distance', String, self.distance_covered_callback)
		self.displacement_sub = rospy.Subscriber('/displacement', String, self.displacement_callback)
		self.speed_sub = rospy.Subscriber('/odom/speed', String, self.speed_callback)
		self.lat_long_sub = rospy.Subscriber('/navsat/fix', NavSatFix, self.lat_long_callback)
		self.easting_northing_sub = rospy.Subscriber('/easting_northing', Float32MultiArray, self.easting_northing_callback)
		self.total_steps_sub = rospy.Subscriber('/steps', Int32, self.total_steps_callback)																

		self.magnetometer_direction_pixmap = QPixmap(self.map_view_package_path+"/assets/red-direction.png")
		self.waypoint_direction_pixmap = QPixmap(self.map_view_package_path+"/assets/white-direction.png")

		uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
		roslaunch.configure_logging(uuid)
		self.launch = roslaunch.parent.ROSLaunchParent(uuid, [self.map_view_package_path+"/launch/map_view.launch"])

		self.stackedWidget = self.findChild(QStackedWidget, 'stackedWidget')
		self.stackedWidget.setCurrentIndex(0)

		self.UserIDLineEdit = self.findChild(QLineEdit, 'UserIDLineEdit')
		self.PasswordLineEdit = self.findChild(QLineEdit, 'PasswordLineEdit')
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
		# oskb phoney-us phoney-de --width 480 --center --start minimized
		self.osk_command = ["oskb", "phoney-us", "phoney-de", "--width", "480", "--center", "--start", "minimized"]
		self.osk_process = QProcess(self)  # Parent to the window
		self.line_edits = [] # List to store all QLineEdit widgets

		# After UI setup, add the line edits to the list
		self.find_line_edits() # Call this after UI is loaded

		for line_edit in self.line_edits:
			line_edit.focusInEvent = lambda event, le=line_edit: self.line_edit_focus_in(event, le)
			line_edit.focusOutEvent = lambda event, le=line_edit: self.line_edit_focus_out(event, le)
		# Connect QProcess signals
		self.osk_process.started.connect(self.osk_started)
		self.osk_process.finished.connect(self.osk_finished)
		self.osk_process.errorOccurred.connect(self.osk_error)  # Handle errors

	def find_line_edits(self):
		for widget in self.findChildren(QLineEdit):
			self.line_edits.append(widget)
			print(f"Found Line Edit: {widget.objectName()}")
			
	def line_edit_focus_in(self, event, line_edit):
		QLineEdit.focusInEvent(line_edit, event)
		self.start_osk()
	
	def line_edit_focus_out(self, event, line_edit):
		QLineEdit.focusOutEvent(line_edit, event)
		self.stop_osk()
		
	def start_osk(self):
		if self.osk_process.state() == QProcess.NotRunning:  # Check if already running
			self.osk_process.start(self.osk_command[0], self.osk_command[1:] if len(self.osk_command) > 1 else []) # Pass command and arguments separately
			print("OSK starting...")
			
	def stop_osk(self):
		if self.osk_process.state() == QProcess.Running:
			self.osk_process.terminate()  # Try to terminate gracefully
			if not self.osk_process.waitForFinished(2000):  # Wait with timeout (2 seconds)
				self.osk_process.kill()  # Force kill if termination fails
				print("OSK forcefully stopped")
			else:
				print("OSK stopped")


	def osk_started(self):
		print("OSK started successfully.")

	def osk_finished(self, exitCode, exitStatus):
		print(f"OSK finished with exit code: {exitCode}, status: {exitStatus}")
		self.osk_process.close()  # Important: close the process after it's finished
		# self.osk_process = QProcess(self) # If you need to restart it later, recreate the QProcess
		# or use self.osk_process.reset() if Qt version supports it.
		# Otherwise you'll get "Process is already running" error

	def osk_error(self, error):
		print(f"OSK error: {error}")
		if error == QProcess.FailedToStart:
			print(f"Error: OSK command '{self.osk_command}' not found or not executable.")

	def scale_widgets(self):
        # Get screen size
		screen_size = QApplication.primaryScreen().availableGeometry()

        # Design-time size
		design_width = 1200
		design_height = 900

		window_height = round(screen.size().height()*0.9)
		window_width = round((window_height*1200)/900)
		self.setFixedSize(window_width, window_height)

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
			##  Get Border Width from stylesheet
			# if widget.objectName == "UserIDLineEdit" :
			try:
				if widget.objectName() == "UserIDLineEdit" or widget.objectName() == "PasswordLineEdit":
					border_width = round(5*scale_factor)
					border_radius = round(10*scale_factor)
					css = qstylizer.style.StyleSheet()
					css[f"QWidget#{widget.objectName()}"]["border-width"].setValue(f"{border_width}px")
					css[f"QWidget#{widget.objectName()}"]["background-color"].setValue("transparent")
					css[f"QWidget#{widget.objectName()}"]["border-style"].setValue("outset")
					css[f"QWidget#{widget.objectName()}"]["border-color"].setValue("yellow")
					css[f"QWidget#{widget.objectName()}"]["border-radius"].setValue(f"{border_radius}px")
					css[f"QWidget#{widget.objectName()}"]["color"].setValue("yellow")
					widget.setStyleSheet(css.toString())

				# css = qstylizer.parser.parse(widget.styleSheet())
				# print(css["QWidget#UserIDLineEdit"].borderWidth)

			except Exception as e:
				print(e)

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
			self.rotated_pixmap_mag = self.magnetometer_direction_pixmap.transformed(QTransform().rotate(msg.data-90))
			self.Current_Direction_Label.setPixmap(self.rotated_pixmap_mag)
			self.Magnetic_Bearing_Label.setText(str(int(msg.data)))
			QtGui.QGuiApplication.processEvents()
		except Exception as e:
			print(e)

	def waypoint_dir_callback(self, msg):
		try: 
			self.rotated_pixmap_way = self.waypoint_direction_pixmap.transformed(QTransform().rotate(360.0-msg.data))
			self.Waypoint_Direction_Label.setPixmap(self.rotated_pixmap_way)
			QtGui.QGuiApplication.processEvents()
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
		if str(self.UserIDLineEdit.text()) == "army" and str(self.PasswordLineEdit.text()) == "1234" :
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
		
		#Continue Updqating UI
		self.timer = QTimer()
		self.timer.timeout.connect(self.updateGUI)
		self.timer.start(1000)
		
	def updateGUI(self):
		# QtGui.QGuiApplication.processEvents()
		QApplication.processEvents()
		print("Updating GUI")

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

