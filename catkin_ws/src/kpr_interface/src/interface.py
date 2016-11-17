#!/usr/bin/python

import rospy
from Tkinter import Tk, Label
from kpr_interface.srv import GetSettings
from kpr_interface.msg import SetSetting

from sensor_msgs.msg import Image
from cucumber_msgs.msg import Cucumber
from rosgraph_msgs.msg import Log

settings_pub = rospy.Publisher('/settings/set', SetSetting, queue_size = 10)

def loadSettings():
	rospy.loginfo('Waiting for setting service...')
	rospy.wait_for_service('settings/getAll')
	try:
		resp = rospy.ServiceProxy('settings/getAll', GetSettings)()
		settings = {}
		
		for setting in resp.settings:
			settings[setting.key] = setting.value
		rospy.loginfo('Settings received')
		
		print settings
		return settings
	except rospy.ServiceException, e:
		rospy.logerror("Service call failed: %s", e)
	
def buildScreen(root, settings):
	root.geometry("{0}x{1}+0+0".format(root.winfo_screenwidth(), root.winfo_screenheight()))

	Label(root, text="Settings:").pack()
	
	for setting in settings:
		Label(root, text="{}: {}".format(setting, settings[setting])).pack()
	
def imageCallback(msg, root):
	a = 0
	
def targetCallback(msg, root):
	a = 0
	
def detectionCallback(msg, root):
	a = 0
	
def diagnosticCallback(msg, log):
	log['text'] += '\n[{}] {}'.format(msg.file, msg.msg)
	
if __name__ == '__main__':
	rospy.init_node('interface')
	root = Tk()
	
	settings = loadSettings()
	image_sub = rospy.Subscriber('/left/image_raw', Image, lambda msg: imageCallback(msg, root))
	#target_sub = rospy.Subscriber('<TODO>', Cucumber, lambda msg: targetCallback(msg, root))
	detect_sub = rospy.Subscriber('/stereo/cucumber', Cucumber, lambda msg: detectionCallback(msg, root))
	log = Label(root, text="Log:")
	log.pack()
	diagnostic_sub = rospy.Subscriber('/rosout', Log, lambda msg: diagnosticCallback(msg, log))
	
	buildScreen(root, settings)
	rospy.loginfo('started')
	# rospy.spin() is not needed in this case
	root.mainloop()
	rospy.loginfo('stopped')
