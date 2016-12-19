#!/usr/bin/env python

import sys, rospy, json, os
from kpr_interface.msg import SetSetting
from kpr_interface.srv import *
from diagnostic_msgs.msg import KeyValue

update_pub = rospy.Publisher('/settings/update', SetSetting, queue_size = 10)

SETTINGS = { }

def update() :
	global update_pub
	
	msg = SetSetting()
	t = getSettings(None)
	msg.size = t.size
	msg.settings = t.settings
	
	update_pub.publish(msg)
	
def getValue(setting) :
	value = '_undefined_'
	if setting in SETTINGS:
		value = SETTINGS[setting]
	return value
	
def setSetting(msg) :
	global SETTINGS
	if (msg.size == 0):
		rospy.logwarn('Received setSetting with size = 0')
		return
	if (msg.size != len(msg.settings)):
		rospy.logwarn('Received setSetting with size that is not equal to the actual size')
		return
	for i in range(msg.size):
		data = msg.settings[i]
		if(data.key not in SETTINGS):
			rospy.logwarn("Received unknown setting '%s'", data.key)
			return
		if (data.key == ''):
			rospy.logwarn('Added a setting with an empty key')
		SETTINGS[data.key] = data.value
		rospy.logdebug("Setting '%s' set to '%s'", data.key, data.value)
	update()
	
def getSettings(req) :
	global SETTINGS
	
	data = []
	for i in SETTINGS:
		setting = KeyValue()
		setting.key = i
		setting.value = SETTINGS[i]
		
		data.append(setting)
	return GetSettingsResponse(len(data), data)
	
def parsePath(path):
	'''
	Parse the filepath and return the absolute position of this file.

	@param path The filepath to parse. 
				When this file starts with a '/', it is treated as absolute, otherwise as relative.
	@return The absolute filepath of the input file
	'''
	if path[0] == '/':
		return path
	else :
		return os.path.join(os.path.dirname(__file__), path)
	
def saveSettings(req):
	'''
	Callback method for the service that saves all the current settings.

	@param req The request message of the service
	@return The Response message of the service
	'''
	global SETTINGS
	try:
		with open(parsePath(req.filePath), 'w') as fp:
			json.dump(SETTINGS, fp)
			rospy.loginfo("Settings saved to %s", req.filePath)
		return SettingsIOResponse(SettingsIOResponse.OK)
	except IOError as e:
		rospy.logerr("Unable to save settings to %s", req.filePath)
		rospy.logerr("-> %s", e)
		return SettingsIOResponse(SettingsIOResponse.ERROR_WRITING_FILE)	

def loadSettings(req):
	'''
	Callback method for the service that loads all the current settings.

	@param req The request message of the service
	@return The Response message of the service
	'''
	global SETTINGS
	try:
		with open(parsePath(req.filePath), 'r') as fp:
			SETTINGS = json.load(fp)
		rospy.loginfo("Settings loaded from %s", req.filePath)
		update()
		return SettingsIOResponse(SettingsIOResponse.OK)	
	except IOError as e:
		rospy.logerr("Unable to load settings from %s", req.filePath)
		rospy.logerr("-> %s", e)
		return SettingsIOResponse(SettingsIOResponse.ERROR_READING_FILE)

def main():
	'''
	The main method of the settingsManager.
	Loads the default settings and starts all the ROS communications.
	'''
	rospy.init_node('settingsManager')
	if (loadSettings(SettingsIORequest("../cfg/settings.json")).success != SettingsIOResponse.OK):
		return
	
	setter_sub = rospy.Subscriber('/settings/set', SetSetting, setSetting)
	get_srv = rospy.Service('/settings/get', GetSetting, lambda req: GetSettingResponse(getValue(req.setting)))
	get_srv = rospy.Service('/settings/getAll', GetSettings, getSettings)
	
	save_srv = rospy.Service('/settings/save', SettingsIO, saveSettings)
	load_srv = rospy.Service('/settings/load', SettingsIO, loadSettings)
	
	rospy.loginfo('started')
	update()
	rospy.spin()
	rospy.loginfo('stopped')
	
if __name__ == '__main__':	
	main()
	


