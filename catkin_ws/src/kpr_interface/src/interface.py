#!/usr/bin/python

import rospy
from Tkinter import *
from kpr_interface.srv import GetSettings
from kpr_interface.msg import SetSetting

settings_pub = rospy.Publisher('/settings/set', SetSetting, queue_size = 10)

def loadSettings():
	rospy.loginfo('[GUI] Waiting for setting service...')
	rospy.wait_for_service('settings/getAll')
	try:
		resp = rospy.ServiceProxy('settings/getAll', GetSettings)()
		settings = {}
		
		for setting in resp.settings:
			settings[setting.key] = setting.value
		rospy.loginfo('[GUI] Settings received')
		
		print settings
		return settings
	except rospy.ServiceException, e:
		rospy.logerror("Service call failed: %s", e)
	
def buildScreen(root, settings):
	root.geometry("{0}x{1}+0+0".format(root.winfo_screenwidth(), root.winfo_screenheight()))

	Label(root, text="Settings:").pack()
	
	for setting in settings:
		Label(root, text="{}: {}".format(setting, settings[setting])).pack()
	
if __name__ == '__main__':
	rospy.init_node('interface')
	settings = loadSettings()
	
	root = Tk()
	buildScreen(root, settings)
	rospy.loginfo('[GUI] started')
	# rospy.spin() is not needed in this case
	root.mainloop()
	rospy.loginfo('[GUI] stopped')
