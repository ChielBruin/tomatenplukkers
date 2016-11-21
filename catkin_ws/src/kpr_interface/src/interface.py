#!/usr/bin/python

import rospy
import cv2
import datetime

from Tkinter import *
from PIL import ImageTk, Image
from kpr_interface.srv import GetSettings
from kpr_interface.msg import SetSetting

from diagnostic_msgs.msg import KeyValue
from sensor_msgs.msg import Image
from cucumber_msgs.msg import Cucumber
from rosgraph_msgs.msg import Log
from ros_faster_rcnn.msg import DetectionFull

settings_pub = rospy.Publisher('/settings/set', SetSetting, queue_size = 10)

IMAGE = None

DETECTION = []

LOG = []
LOG_COLORS = {
	Log.DEBUG : 'blue',
	Log.INFO  : 'black',
	Log.WARN  : 'orange',
	Log.ERROR : 'red',
	Log.FATAL : 'purple'
}

SETTINGS = {}

TARGET = None

root = Tk()	

def loadSettings(frame):
	rospy.loginfo('Waiting for setting service...')
	rospy.wait_for_service('settings/getAll')
	try:
		resp = rospy.ServiceProxy('settings/getAll', GetSettings)()
		settings = {}
		
		for setting in resp.settings:
			settings[setting.key] = setting.value
		rospy.loginfo('Settings received')
		
		displaySettings(frame, settings)
	except rospy.ServiceException, e:
		rospy.logerror("Service call failed: %s", e)

def saveSettings(e):
	msg = SetSetting()
	msg.size = len(e)
	data = []
	map(lambda entry: data.append(KeyValue(entry, e[entry].get())), e)
	msg.settings = data
	settings_pub.publish(msg)
	
def displaySettings(screen, settings):
	screen.winfo_children()[1].destroy()
	if (len(screen.winfo_children()) > 1):
		screen.winfo_children()[1].destroy()
	container = Frame(screen, width = 150)
	container.pack()
	entries = {}
	for i, setting in enumerate(settings):
		Label(container, text="{}:".format(setting)).grid(column = 0, row = i)
		e = Entry(container)
		e.insert(0, settings[setting])
		e.grid(column = 1, row = i)
		entries[setting] = e
	Button(screen, text = 'save', command = lambda : saveSettings(entries)).pack()
		
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
	settings = Frame(root)
	settings.grid(column=2, row = 0)	
	Label(settings, text="Settings:").pack()
	Frame(settings).pack() # settings container
	
	middle = Frame(root)
	middle.grid(column=1, row = 0)
	
	# Create image label
	imageContainer = Label(middle)
	imageContainer.grid()

	image = None #ImageTk.PhotoImage()
	imageContainer.image = image
	
	return (lbox, settings, image)

def settingsCallback(msg):
	global SETTINGS		
	for setting in msg.settings:
		SETTINGS[setting.key] = setting.value
	
def updateImage(root):
	global IMAGE, TARGET
	#global newImage, newDetection, newTarget 
	#if not (newImage or newDetection or newTarget):
		#return
	#newImage = False
	#newDetection = False
	#newTarget = False
	a = 0

def getTime(stamp) :
	return datetime.datetime.fromtimestamp(
        int(stamp.secs)
    ).strftime('%H:%M:%S')

def updateLog(root):
	global LOG_COLORS, LOG
	root.after(1000, updateLog, root)
	if not LOG:
		return
	
	for msg in LOG:
		tmp = '{} [{}] {}'.format(getTime(msg.header.stamp), msg.file, msg.msg)
		root.insert(END, tmp)	# TODO colors LOG_COLORS[msg.level], wraplength = 400
		root.itemconfig(END, {'fg': LOG_COLORS[msg.level]})
	LOG = []
	
def updateSettings(root):
	global SETTINGS
	root.after(1000, updateSettings, root)
	if not SETTINGS:
		return
	displaySettings(root, SETTINGS)
	SETTINGS = []
	
def detectionCallback(msg):
	global IMAGE
	IMAGE = msg
    
if __name__ == '__main__':
	rospy.init_node('interface')
	(logRoot, settingsRoot, imageRoot) = buildScreen(root)
	
	image_sub = rospy.Subscriber('/rcnn/res/full', DetectionFull, detectionCallback)
	#target_sub = rospy.Subscriber('<TODO>', Cucumber, lambda msg: TARGET = msg)
	diagnostic_sub = rospy.Subscriber('/rosout', Log, lambda msg: LOG.append(msg))
	settings_sub = rospy.Subscriber('/settings/update', SetSetting, settingsCallback)
	
	rospy.loginfo('started')
	
	loadSettings(settingsRoot)
	
	root.after(500, updateImage, imageRoot)
	root.after(1000, updateLog, logRoot)
	root.after(1000, updateSettings, settingsRoot)
	
	root.mainloop()
			
	rospy.loginfo('stopped')
