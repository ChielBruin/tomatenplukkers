#!/usr/bin/env python

import sys, rospy
from kpr_interface.msg import SetSetting
from kpr_interface.srv import *
from diagnostic_msgs.msg import KeyValue

update_pub = rospy.Publisher('/settings/update', SetSetting, queue_size = 10)

SETTINGS = {
	'minWeight' : '300',
	'maxWeight'	: '400',
	'armSpeed'	: '100',
}

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
	
	for i in range(msg.size):
		data = msg.settings[i]
		SETTINGS[data.key] = data.value
		rospy.logdebug("[Settings manager] '%s' set to '%s'", data.key, data.value)
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
		

if __name__ == '__main__':
	rospy.init_node('settingsManager')
	setter_sub = rospy.Subscriber('/settings/set', SetSetting, setSetting)
	get_srv = rospy.Service('/settings/get', GetSetting, lambda req: GetSettingResponse(getValue(req.setting)))
	get_srv = rospy.Service('/settings/getAll', GetSettings, getSettings)
	
	rospy.loginfo('[Settings manager] started')
	update()
	rospy.spin()
	rospy.loginfo('[Settings manager] stopped')
	


