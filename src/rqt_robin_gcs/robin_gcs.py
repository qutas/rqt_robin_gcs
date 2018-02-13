import os
import math
import rospkg
import rospy

from qt_gui.plugin import Plugin
from python_qt_binding import loadUi
from python_qt_binding.QtWidgets import QWidget

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

		self._widget.button_cal_accel.clicked.connect(self.button_cal_accel_pressed)
		self._widget.button_cal_gyro.clicked.connect(self.button_cal_gyro_pressed)
		self._widget.button_cal_esc.clicked.connect(self.button_cal_esc_pressed)
		self._widget.button_reboot_bootloader.clicked.connect(self.button_reboot_bootloader_pressed)
		self._widget.button_reboot_system.clicked.connect(self.button_reboot_system_pressed)
		self._widget.button_update_namespace.clicked.connect(self.button_update_namespace_pressed)
		self._widget.button_write_eeprom.clicked.connect(self.button_write_eeprom_pressed)

		#mavros.set_namespace("/mavros")
		self.update_namespace()

	def shutdown_plugin(self):
		pass

	def save_settings(self, plugin_settings, instance_settings):
		# TODO save intrinsic configuration, usually using:
		# instance_settings.set_value(k, v)
		pass

	def restore_settings(self, plugin_settings, instance_settings):
		# TODO restore intrinsic configuration, usually using:
		# v = instance_settings.value(k)
		pass

	#def trigger_configuration(self):
		# Comment in to signal that the plugin has a way to configure
		# This will enable a setting button (gear icon) in each dock widget title bar
		# Usually used to open a modal configuration dialog

	def button_cal_accel_pressed(self):
		self.call_command(241, 0, 0, 0, 0, 1, 0, 0)
		rospy.loginfo("DEBUG: Cal accel button pressed!")

	def button_cal_gyro_pressed(self):
		self.call_command(241, 1, 0, 0, 0, 0, 0, 0)
		rospy.loginfo("DEBUG: Cal gyro button pressed!")

	def button_cal_esc_pressed(self):
		rospy.loginfo(param_set("DO_ESC_CAL", 1))
		rospy.logwarn("If calibrate ESC parameter set, write params and reboot to complete calibration")
		rospy.logwarn("---\n\nMake sure props are detached!\n\n---")
		rospy.loginfo("DEBUG: Cal esc button pressed!")

	def button_reboot_bootloader_pressed(self):
		self.call_command(246, 3, 0, 0, 0, 0, 0, 0)
		rospy.loginfo("DEBUG: Reboot bootloader button pressed!")

	def button_reboot_system_pressed(self):
		self.call_command(246, 1, 0, 0, 0, 0, 0, 0)
		rospy.loginfo("DEBUG: Reboot system button pressed!")

	def button_update_namespace_pressed(self):
		self.update_namespace()
		rospy.loginfo("DEBUG: Update service name button pressed!")

	def button_write_eeprom_pressed(self):
		self.call_command(245, 1, 0, 0, 0, 0, 0, 0)
		rospy.loginfo("DEBUG: Write EEPROM button pressed!")

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
			fault(ex)

	def check_ret(self,ret):
		if not ret.success:
			rospy.logerr("Request failed. Check mavros logs. ACK: %i" % ret.result)

		rospy.loginfo("Command ACK: %i" % ret.result)

	def update_namespace(self):
		ns = self._widget.textbox_namespace.text()
		mavros.set_namespace(ns)


