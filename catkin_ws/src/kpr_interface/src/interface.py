#!/usr/bin/python

import rospy
import cv2
import datetime
import os

from Tkinter import *
from PIL import ImageTk
import PIL.Image
from cv_bridge import CvBridge, CvBridgeError

from kpr_interface.srv import GetSettings, SettingsIO
from kpr_interface.msg import SetSetting
from diagnostic_msgs.msg import KeyValue
from sensor_msgs.msg import Image
from cucumber_msgs.msg import Cucumber
from rosgraph_msgs.msg import Log
from ros_faster_rcnn.msg import DetectionFull

settings_pub = rospy.Publisher('/settings/set', SetSetting, queue_size = 10)

IMAGE = [False, None, None]
SETTINGS = {}
LOG = []
LOG_COLORS = {
	Log.DEBUG : 'blue',
	Log.INFO  : 'black',
	Log.WARN  : 'orange',
	Log.ERROR : 'red',
	Log.FATAL : 'purple'
}

# Converts a ROS timestamp to a readable string of the time
# stamp: The ROS timestamp to convert
# returns: A readable string of the timestamp
def getTime(stamp) :
	return datetime.datetime.fromtimestamp(int(stamp.secs)).strftime('%H:%M:%S')
	
# Requests settings from the settings manager and displays them.
# root: The node to display the settings on
def requestSettings(root):
	rospy.loginfo('Waiting for setting service...')
	rospy.wait_for_service('settings/getAll')
	try:
		resp = rospy.ServiceProxy('settings/getAll', GetSettings)()
		settings = {}
		
		for setting in resp.settings:
			settings[setting.key] = setting.value
		rospy.loginfo('Settings received')
		
		displaySettings(root, settings)
	except rospy.ServiceException, e:
		rospy.logerror("Service call failed: %s", e)

# Saves settings for each entry in the list to the settings manager
# entries: A dictionary with each setting mapped to the Entry that contains the setting
def saveSettings(entries):
	msg = SetSetting()
	msg.size = len(entries)
	data = []
	map(lambda e: data.append(KeyValue(e, entries[e].get())), entries) # convert settings to KeyValue messages
	msg.settings = data
	try:
		rospy.wait_for_service('settings/getAll', timeout = 5)
		rospy.loginfo("Settings saved")
		settings_pub.publish(msg)
	except:
		rospy.logerr("Settings manager is offline, settings are not saved")		
	
def storeSettings():
	try:
		path = "/home/chiel/Downloads/banaan.json"
		rospy.wait_for_service('settings/save', timeout = 5)
		rospy.ServiceProxy('settings/save', SettingsIO)(path)
	except:
		rospy.logerr("Error while loading settings")
		
def loadSettings():
	try:
		path = "/home/chiel/Downloads/banaan.json"
		rospy.wait_for_service('settings/load', timeout = 5)
		rospy.ServiceProxy('settings/load', SettingsIO)(path)
	except:
		rospy.logerr("Error while loading settings")
	
# Displays a list of all settings on the screen
# root: The node to display all settings on
# settings: A dictionary with all settings to display
def displaySettings(root, settings):
	root.winfo_children()[1].destroy()			# Remove the old container
	if (len(root.winfo_children()) > 1):
		root.winfo_children()[1].destroy()		# Remove the save button if present
	container = Frame(root, width = 150)
	container.pack()
	entries = {}
	for i, setting in enumerate(settings):
		Label(container, text="{}:".format(setting)).grid(column = 0, row = i)
		e = Entry(container)
		e.insert(0, settings[setting])
		e.grid(column = 1, row = i)
		entries[setting] = e
		
	Button(root, text = 'save', command = lambda : saveSettings(entries)).pack()
		
# Build the initial screen with the root nodes for each display.
# root: The node to display the interface on
# returns: A triple containing the logger root, settings root and image root respectively
def buildScreen(root):
	width = root.winfo_screenwidth()
	height = root.winfo_screenheight()
	root.geometry("{0}x{1}+0+0".format(width, height))

	# Create log frame
	log = Frame(root)
	log.grid(column = 0, row = 0)
	Label(log, text="LOG").pack()
	vsbar = Scrollbar(log)
	vsbar.pack(side=RIGHT, fill=Y)
	hsbar = Scrollbar(log, orient=HORIZONTAL)
	hsbar.pack(side=BOTTOM, fill=X)
	lbox = Listbox(log, width=50, height = 30, yscrollcommand=vsbar.set, xscrollcommand=hsbar.set)
	lbox.pack(fill=None, expand=False)
	lbox.pack_propagate(0)
	vsbar.config(command=lbox.yview)
	hsbar.config(command=lbox.xview)
	
	# Create settings frame
	settingsFrame = Frame(root)
	settingsFrame.grid(column=2, row = 0)	
	settings = Frame(settingsFrame)
	settings.pack()
	Label(settingsFrame, text="Store and load settings to/from file").pack()
	loadSave = Frame(settingsFrame)
	loadSave.pack()
	Button(loadSave, text = 'Store', command = lambda: storeSettings()).grid(column=0, row = 0)
	Button(loadSave, text = 'Load', command = lambda: loadSettings()).grid(column=1, row = 0)
	Label(settings, text="Settings:").pack()
	Frame(settings).pack() # settings container
	
	middle = Frame(root)
	middle.grid(column=1, row = 0)
	
	# Create image canvas
	canvas = Canvas(middle, width = 500, height = 400)

	image = ImageTk.PhotoImage(PIL.Image.open(os.path.join(os.path.dirname(__file__), "logo.png")))
	canvas.create_image(250, 200, image=image)
	canvas.pack()
	
	return (lbox, settings, (image, canvas))

# Callback for receiving updated settings
# msg: A SetSettings message containing the new values
def settingsCallback(msg):
	global SETTINGS		
	for setting in msg.settings:
		SETTINGS[setting.key] = setting.value

# Callback for receiving new images to display
# img: The new CV image to display
# cv_bridge: A cv_bridge instance that is used to convert the image
def imageCallback(img, cv_bridge):
	global IMAGE
	IMAGE[0] = True
	IMAGE[1] = cv_bridge.imgmsg_to_cv2(img)

# Callback for receiving a new target to display
# img: The new target to display
def targetCallback(msg):
	global IMAGE
	IMAGE[0] = True
	IMAGE[2] = msg

# Calculates the 2D position of the target from its location.
# target: The target object
# returns: The 2D pixel location of the target
def calculateTarget(target):
	x = target.image_stem_position[0]
	y = target.image_stem_position[1]
	return (x-5, y-5, x+5, y+5)

# Displays the target position.
# position: The 2D position of the target in the image
# root: The node to display the target on
def displayTarget(position, canvas):
	canvas.create_oval(position, fill='red')
	
# Displays the image with detections.
# image: the image to display
# root: The node to display the image on
def displayImage(image, canvas):
	img = PIL.Image.fromarray(cv2.cvtColor(image, cv2.COLOR_BGR2RGB))
	img = img.resize((500, 400), PIL.Image.ANTIALIAS)
	tk_img = ImageTk.PhotoImage(img)
	canvas.create_image(250,200,image=tk_img)
	return tk_img
	
# Updates the displayed image and target.
# root: The node to display the image on
def updateImage(root, canvas):
	global IMAGE, imageRef
	root.after(500, updateImage, root, canvas)
	if not IMAGE[0]:
		return
	canvas.delete(ALL)
	IMAGE[0] = False
	
	if IMAGE[1] is not None:
		imageRef = displayImage(IMAGE[1], canvas)
	if IMAGE[2] is not None:
		targetpos = calculateTarget(IMAGE[2])
		displayTarget(targetpos, canvas)

# Updates the log display by adding the new log entries.
# root: The node that must contain all entries
def updateLog(root):
	global LOG_COLORS, LOG
	root.after(1000, updateLog, root)
	if not LOG:
		return
	
	for msg in LOG:
		tmp = '{} [{}] {}'.format(getTime(msg.header.stamp), msg.file, msg.msg)
		root.insert(END, tmp)
		if msg.level in LOG_COLORS:
			root.itemconfig(END, {'fg': LOG_COLORS[msg.level]})		# Set the color of the entry
		else:
			rospy.logwarn("Unknown log level %d", msg.level)
	LOG = []
	
# Updates the settings display
# root: The node to display the settings on
def updateSettings(root):
	global SETTINGS
	root.after(1000, updateSettings, root)
	if not SETTINGS:
		return
	displaySettings(root, SETTINGS)
	SETTINGS = {}
   
# Main method
# Starts all the ROS subscribers and the TKinter loop 
if __name__ == '__main__':
	rospy.init_node('interface')
	cv_bridge = CvBridge()
	root = Tk()
	root.wm_title("KasPR | Interface")
	(logRoot, settingsRoot, (imageRef, imageRoot)) = buildScreen(root)
	
	image_sub = rospy.Subscriber('/rcnn/res/full', DetectionFull, lambda msg: imageCallback(msg.image, cv_bridge))
	#image_sub = rospy.Subscriber('/rcnn/image_raw', Image, lambda msg: imageCallback(msg, cv_bridge)) # Use this to use the imageLoader.py to test
	target_sub = rospy.Subscriber('/target', Cucumber, targetCallback)
	diagnostic_sub = rospy.Subscriber('/rosout', Log, lambda msg: LOG.append(msg))
	settings_sub = rospy.Subscriber('/settings/update', SetSetting, settingsCallback)
	
	rospy.loginfo('started')
	
	requestSettings(settingsRoot)
	
	root.after(500, updateImage, root, imageRoot)
	root.after(1000, updateLog, logRoot)
	root.after(1000, updateSettings, settingsRoot)
	
	root.mainloop()
			
	rospy.loginfo('stopped')
