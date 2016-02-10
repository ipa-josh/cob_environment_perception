#!/usr/bin/python


class Feature:
	def __init__(self, ft, trans, pos):
		self.ft = ft
		self.transition = trans
		self.position = pos
		
	def inject(self, ctxt):
		ctxt.add_hypo(self.transition, self.position)
		
