#!/usr/bin/env python

from enum import Enum

class MoveStatus(Enum):
	'''
	Enum class containing the return values for the moveArmTo function.
	'''
	MOVE_OK = 0
	PLAN_ERROR = 1
	MOVE_ERROR = 2
