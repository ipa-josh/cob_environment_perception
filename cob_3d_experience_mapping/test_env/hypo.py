#!/usr/bin/python


class Hypothesis:
	def __init__(self, trans, pose):
		self.trans = trans
		self.pose = pose
		
	def update(self, action):
		return
		
	def p(self):
		return 0.
		
	def merge(self, other):
		return False
