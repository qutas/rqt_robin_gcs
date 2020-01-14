import os
import math
import rospkg
import rospy

from qt_gui.plugin import Plugin
from python_qt_binding import loadUi
from python_qt_binding.QtWidgets import QWidget

from rqt_robin_gcs.robin_gcs_options import SimpleSettingsDialog

import mavros
from mavros.utils import *
from mavros.param import *
from mavros import command

class RobinGCS(Plugin):
	def __init__(self, context):
		super(RobinGCS, self).__init__(context)
		# Give QObjects reasonable names
		self.setObjectName('RobinGCS')
		rp = rospkg.RosPack()

		# Process standalone plugin command-line arguments
		#from argparse import ArgumentParser
		#parser = ArgumentParser()
		# Add argument(s) to the parser.
		#parser.add_argument("-q", "--quiet", action="store_true",
		#              dest="quiet",
		#              help="Put plugin in silent mode")
		#args, unknowns = parser.parse_known_args(context.argv())
		#if not args.quiet:
		#    print 'arguments: ', args
		#    print 'unknowns: ', unknowns

		# Create QWidget
		self._widget = QWidget()
		# Get path to UI file which is a sibling of this file
		# in this example the .ui and .py file are in the same folder
		ui_file = os.path.join(rp.get_path('rqt_robin_gcs'), 'resource', 'RobinGCS.ui')
		# Extend the widget with all attributes and children from UI file
		loadUi(ui_file, self._widget)
		# Give QObjects reasonable names
		self._widget.setObjectName('RobinGCSUi')
		# Show _widget.windowTitle on left-top of each plugin (when
		# it's set in _widget). This is useful when you open multiple
		# plugins at once. Also if you open multiple instances of your
		# plugin at once, these lines add number to make it easy to
		# tell from pane to pane.
		if context.serial_number() > 1:
			self._widget.setWindowTitle(self._widget.windowTitle() + (' (%d)' % context.serial_number()))
		# Add widget to the user interface
		context.add_widget(self._widget)

		self.mavros_namespace = "/mavros"

		self._widget.button_mixer_set.clicked.connect(self.button_mixer_set_pressed)
		self._widget.button_cal.clicked.connect(self.button_cal_pressed)
		self._widget.button_rc_mode_2.clicked.connect(self.button_rc_mode_2_pressed)
		self._widget.button_motor_test.clicked.connect(self.button_motor_test_pressed)
		self._widget.button_reboot_bootloader.clicked.connect(self.button_reboot_bootloader_pressed)
		self._widget.button_reboot_system.clicked.connect(self.button_reboot_system_pressed)
		self._widget.button_write_eeprom.clicked.connect(self.button_write_eeprom_pressed)

		#mavros.set_namespace("/mavros")
		#self.update_namespace()

	def shutdown_plugin(self):
		pass

	def save_settings(self, plugin_settings, instance_settings):
		instance_settings.set_value("namespace", self.mavros_namespace)

	def restore_settings(self, plugin_settings, instance_settings):
		ns = instance_settings.value("namespace")
		if ns:
			self.mavros_namespace = str(ns)

		mavros.set_namespace(self.mavros_namespace)

	def trigger_configuration(self):
		"""Present the user with a dialog for choosing the topic to view,
		the data type, and other settings used to generate the HUD.
		This displays a SimpleSettingsDialog asking the user to choose
		the settings as desired.

		This method is blocking"""

		dialog = SimpleSettingsDialog(title='Quaternion View Options')
		dialog.add_lineedit("namespace", str(self.mavros_namespace), "Namespace")

		settings = dialog.get_settings();
		if settings is not None:
			for s in settings:
				if s[0] == "namespace":
					self.mavros_namespace = str(s[1])

		mavros.set_namespace(self.mavros_namespace)

	def button_cal_pressed(self):
		param1 = 0
		param2 = 0
		param3 = 0
		param4 = 0
		param5 = 0
		param6 = 0
		param7 = 0

		cal = self._widget.combo_cal.currentText()

		if cal == "GYROSCOPE":
			param1 = 1
		elif cal == "MAGNETOMETER":
			param2 = 1
		elif cal == "GROUND PRESSURE":
			param3 = 1
		elif cal == "RC":
			param4 = 1
		elif cal == "ACCELEROMETER":
			param5 = 1
		elif cal == "LEVEL HORIZON":
			param5 = 2
		elif cal == "INTERFERENCE":
			param6 = 1
		elif cal == "ESC":
			param7 = 1
		elif cal == "BAROMETER":
			param7 = 3

		self.call_command(241, param1, param2, param3, param4, param5, param6, param7)

	def button_rc_mode_2_pressed(self):
		params = [["RC_MAP_ROLL", 1],
				  ["RC_MAP_PITCH", 2],
				  ["RC_MAP_THROTTLE", 3],
				  ["RC_MAP_YAW", 4],
				  ["RC_MAP_MODE_SW", 5]]
		for p in params:
			try:
				rospy.loginfo(p[0] + ": " + str(param_set(p[0], int(p[1]))))
			except IOError as e:
				rospy.logerr(e)

		rospy.logdebug("Set RC mode 2 button pressed!")

	def button_motor_test_pressed(self):
		self.call_command(209, 255, 0, 0.4, 1.5, 0, 0, 0)

	def button_reboot_bootloader_pressed(self):
		self.call_command(246, 3, 0, 0, 0, 0, 0, 0)

	def button_reboot_system_pressed(self):
		self.call_command(246, 1, 0, 0, 0, 0, 0, 0)

	def button_update_namespace_pressed(self):
		self.update_namespace()

	def button_write_eeprom_pressed(self):
		self.call_command(245, 1, 0, 0, 0, 0, 0, 0)

	def call_command(self, cmd, p1, p2, p3, p4, p5, p6, p7):
		try:
			ret = command.long(broadcast=0,
								command=cmd, confirmation=int(1),
								param1=p1,
								param2=p2,
								param3=p3,
								param4=p4,
								param5=p5,
								param6=p6,
								param7=p7)

			self.check_ret(ret)
		except rospy.ServiceException as ex:
			rospy.logerr(ex)

	def button_mixer_set_pressed(self):
		param_id = "SYS_AUTOSTART"
		mixer_str = str(self._widget.combo_mixer_list.currentText())
		mixer_val = 0;

		if mixer_str == "Generic Plane":
			mixer_val = 2100
		elif mixer_str == "Quadrotor x4":
			mixer_val = 4001
		elif mixer_str == "Quadrotor +4":
			mixer_val = 5001
		elif mixer_str == "Hexarotor x4":
			mixer_val = 6001

		try:
			rospy.loginfo(param_set(param_id, mixer_val))
		except IOError as e:
			rospy.logerr(e)

		rospy.logdebug("Set param button pressed!")

	def check_ret(self,ret):
		if not ret.success:
			rospy.logerr("Request failed. Check mavros logs. ACK: %i" % ret.result)

		rospy.loginfo("Command ACK: %i" % ret.result)



