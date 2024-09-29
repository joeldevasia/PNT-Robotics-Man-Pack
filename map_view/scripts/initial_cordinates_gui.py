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
		self.setWindowTitle("Insert Initial Coordinates")

		# setting geometry to the window
		self.setGeometry(100, 100, 300, 100)

		# creating a group box
		self.formGroupBox = QGroupBox()

		# creating a line edit
		self.initial_latiude = QLineEdit()
		self.initial_longitude = QLineEdit()

		self.rospkg_path = rospkg.RosPack()
		self.package_path = self.rospkg_path.get_path('map_view')
		self.initial_coordinates_file_path = self.package_path+"/config/initial_coordinates.yaml"

		# calling the method that create the form
		self.createForm()

		# creating a dialog button for ok and cancel
		# QDialogButtonBox.Cancel
		self.buttonBox = QDialogButtonBox()

		startButton = QPushButton(self.tr("&Save and Start"))
		startButton.clicked.connect(self.save_cordinates
							  )
		loadButton = QPushButton(self.tr("&Load"))
		loadButton.clicked.connect(self.load_cordinates)
		
		self.buttonBox.addButton(QDialogButtonBox.Cancel)
		self.buttonBox.addButton(loadButton, QDialogButtonBox.ActionRole)
		self.buttonBox.addButton(startButton, QDialogButtonBox.ActionRole)

		# adding action when form is accepted
		self.buttonBox.accepted.connect(self.save_cordinates)
        

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


		# setting layout
		self.formGroupBox.setLayout(layout)
		
	def load_cordinates(self):
        # Load and parse the map.yaml file
		# print(os.getcwd()+'/src/carpack_map_view/config/map.yaml')
		with open(self.initial_coordinates_file_path, 'r') as file:
			yaml_data = yaml.safe_load(file)
			latitude = yaml_data['local_xy_origins'][0]['latitude']
			longitude = yaml_data['local_xy_origins'][0]['longitude']
			self.initial_latiude.setText(str(latitude))
			self.initial_longitude.setText(str(longitude))
			file.close()
			
	def save_cordinates(self):
        # Load and parse the map.yaml file
		with open(self.initial_coordinates_file_path, 'r') as file:
			yaml_data = yaml.safe_load(file)
		try:
			f = float(self.initial_latiude.text())
			f = float(self.initial_longitude.text())
		except Exception as e:
			print(e)
			return
		with open(self.initial_coordinates_file_path, 'w') as file:
			yaml_data['local_xy_origins'][0]['latitude'] = float(self.initial_latiude.text())
			yaml_data['local_xy_origins'][0]['longitude'] = float(self.initial_longitude.text())
			yaml.dump(yaml_data, file)

		
		self.close()
		uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
		roslaunch.configure_logging(uuid)
		launch = roslaunch.parent.ROSLaunchParent(uuid, [self.package_path+"/launch/map_view.launch"])
		launch.start()
		rospy.loginfo("started")
		rospy.sleep(10.0)
		rospy.spin()
		launch.shutdown()
		


# main method
if __name__ == '__main__':
	rospy.init_node('initial_cordinates_gui', anonymous=True)

	# create pyqt5 app
	app = QApplication(sys.argv)

	# create the instance of our Window
	window = Window()

	# showing the window
	window.show()

	# rospy.signal_shutdown("System Exited")

	# start the app
	sys.exit(app.exec())
