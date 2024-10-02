#!/usr/bin/env python3
from PyQt5.QtWidgets import *
import sys
import yaml
import os
import rospy
import rospkg
import roslaunch

# creating a class
# that inherits the QDialog class
class Window(QDialog):

	# constructor
	def __init__(self):
		super(Window, self).__init__()

		# setting window title
		self.setWindowTitle("Insert Configuration Parameter")

		# setting geometry to the window
		self.setGeometry(100, 100, 300, 100)

		# creating a group box
		self.formGroupBox = QGroupBox()

		# creating a line edit
		self.initial_latiude = QLineEdit()
		self.initial_longitude = QLineEdit()
		self.stride_length = QLineEdit()

		self.rospkg_path = rospkg.RosPack()
		self.map_view_package_path = self.rospkg_path.get_path('map_view')
		self.nda_bot_package_path = self.rospkg_path.get_path('nda_bot')
		self.initial_coordinates_file_path = self.map_view_package_path+"/config/initial_coordinates.yaml"
		self.config_file_path = self.nda_bot_package_path+"/config/config.yaml"

		# calling the method that create the form
		self.createForm()

		# creating a dialog button for ok and cancel
		# QDialogButtonBox.Cancel
		self.buttonBox = QDialogButtonBox()

		startButton = QPushButton(self.tr("&Save and Start"))
		startButton.clicked.connect(self.save_parameters
							  )
		loadButton = QPushButton(self.tr("&Load"))
		loadButton.clicked.connect(self.load_parameters)
		
		self.buttonBox.addButton(QDialogButtonBox.Cancel)
		self.buttonBox.addButton(loadButton, QDialogButtonBox.ActionRole)
		self.buttonBox.addButton(startButton, QDialogButtonBox.ActionRole)

		# adding action when form is accepted
		self.buttonBox.accepted.connect(self.save_parameters)
        

		# adding action when form is rejected
		self.buttonBox.rejected.connect(self.reject)

		# creating a vertical layout
		mainLayout = QVBoxLayout()

		# adding form group box to the layout
		mainLayout.addWidget(self.formGroupBox)

		# adding button box to the layout
		mainLayout.addWidget(self.buttonBox)

		# setting lay out
		self.setLayout(mainLayout)

	# create form method
	def createForm(self):

		# creating a form layout
		layout = QFormLayout()

		# adding rows
		# for name and adding input text
		layout.addRow(QLabel("Initial Latitude"), self.initial_latiude)

		# for degree and adding combo box
		layout.addRow(QLabel("Initial Longitude"), self.initial_longitude)


		layout.addRow(QLabel("Stride length(2 Steps)"), self.stride_length)


		# setting layout
		self.formGroupBox.setLayout(layout)
		
	def load_parameters(self):
        # Load and parse the map.yaml file
		# print(os.getcwd()+'/src/carpack_map_view/config/map.yaml')
		with open(self.initial_coordinates_file_path, 'r') as file:
			yaml_data = yaml.safe_load(file)
			latitude = yaml_data['local_xy_origins'][0]['latitude']
			longitude = yaml_data['local_xy_origins'][0]['longitude']
			self.initial_latiude.setText(str(latitude))
			self.initial_longitude.setText(str(longitude))
			file.close()
		
		with open(self.config_file_path, 'r') as file:
			yaml_data = yaml.safe_load(file)
			pulse_per_meter = yaml_data['configuration'][0]['stride_length']
			self.stride_length.setText(str(pulse_per_meter))
			file.close()
			
	def save_parameters(self):
        # Load and parse the map.yaml file
		with open(self.initial_coordinates_file_path, 'r') as file:
			coordinates_yaml_data = yaml.safe_load(file)

		with open(self.config_file_path, 'r') as file:
			config_yaml_data = yaml.safe_load(file)
		try:
			f = float(self.initial_latiude.text())
			f = float(self.initial_longitude.text())
			i = float(self.stride_length.text())
		except Exception as e:
			print(e)
			return
		with open(self.initial_coordinates_file_path, 'w') as file:
			coordinates_yaml_data['local_xy_origins'][0]['latitude'] = float(self.initial_latiude.text())
			coordinates_yaml_data['local_xy_origins'][0]['longitude'] = float(self.initial_longitude.text())
			yaml.dump(coordinates_yaml_data, file)

		with open(self.config_file_path, 'w') as file:
			config_yaml_data['configuration'][0]['stride_length'] = float(self.stride_length.text())
			yaml.dump(config_yaml_data, file)

		
		self.close()
		uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
		roslaunch.configure_logging(uuid)
		launch = roslaunch.parent.ROSLaunchParent(uuid, [self.map_view_package_path+"/launch/map_view.launch"])
		launch.start()
		rospy.loginfo("started")
		rospy.sleep(10.0)
		rospy.spin()
		launch.shutdown()
		


# main method
if __name__ == '__main__':
	rospy.init_node('initial_parameters_gui', anonymous=True)

	# create pyqt5 app
	app = QApplication(sys.argv)

	# create the instance of our Window
	window = Window()

	# showing the window
	window.show()

	# rospy.signal_shutdown("System Exited")

	# start the app
	sys.exit(app.exec())
