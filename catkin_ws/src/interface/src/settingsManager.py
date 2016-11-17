#!/usr/bin/env python

import sys, rospy
from interface.msg import SetSetting, GetSetting, GetSettings

SETTINGS = {
	'minWeight' : '300',
	'maxWeight'	: '400',
	'armSpeed'	: '100',
}

def getSetting(setting) :
	value = '_undefined_'
	if setting in SETTINGS:
		value = SETTINGS[setting]
	return value
	
def setSetting(msg) :
	global SETTINGS
	
	for i in range(msg.size):
		data = msg.data[i]
		SETTINGS[data.Key] = data.Value
	
def getSettings(req) :
	global SETTINGS
	
	data = []
	for i in SETTINGS:
		setting = KeyValue()
		setting.key = i
		setting.value = SETTINGS[i]
		
		data.append(setting)
	return GetSettingsResponse(len(data), data)
		

if __name__ == '__main__':
	rospy.init_node('settingsManager')
	setter_sub = rsopy.Subscriber('/settings/set', SetSetting, setSetting)
	get_srv = rospy.Service('/settings/get', GetSetting, lambda req: GetSettingResponse(getValue(req.setting)))
	get_srv = rospy.Service('/settings/getAll', GetSettings, getSettings)
	rospy.spin()


