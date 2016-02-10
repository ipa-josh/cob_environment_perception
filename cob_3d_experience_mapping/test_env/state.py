#!/usr/bin/python


class State:
	def __init__(self):
		self.transitions = []
		
	def add_transition(self, tgt):
		self.transitions.append( Transition(self, tgt) )
		
