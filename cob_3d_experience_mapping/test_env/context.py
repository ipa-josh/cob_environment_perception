#!/usr/bin/python

from state import State
from hypo import Hypothesis
from transition import Transition

class Robot:
	def __init__(self, wheelbase, std_vel, std_steer):
		self.wheelbase = wheelbase
		self.std_vel = std_vel
		self.std_steer = std_steer

class Context:
	def __init__(self, robot):
		self.robot = robot
		self.virtual_hypo = None
		self.features = {}
		self.states = [State()]
		self.hypos = []
		self.actions_all = []
		self.hypos_new = []
		
		self.new_virtual_hypo()
		
	def new_virtual_hypo(self):
		if self.virtual_hypo != None:
			self.state.append( self.virtual_hypo )
		self.virtual_hypo = Hypothesis(Transition(self.current_state(), State()))
		self.hypos.append( self.virtual_hypo )
		
	def on_feature(self, ft):
		if ft.identifier in self.features:
			for f in self.features[ft.identifier]:
				f.inject(self)
		else:
			self.features[ft.identifier] = []
			
		self.features[ft.identifier].append(Feature(ft, self.current_pose()))
		
	def on_action(self, action):
		for h in self.hypos_new:
			h.p *= len(self.hypos)
		self.hypos_new = []
		
		last_h = self.current_state()
		
		for h in self.hypos:
			h.update(action)
			
		for i in xrange(len(self.hypos)):
			for j in range(i+1, len(self.hypos)):
				if self.hypos[i].merge(self.hypos[j]):
					self.hypos.remove(j)
					j-=1
			
		self.hypos = sorted(self.hypos, key=methodcaller('p'))
		self.self.actions_all.append(action)
		
		if last_h != self.current_state():
			print "we have new current state"
			self.virtual_hypo = None
			self.new_virtual_hypo()
		#elif self.virtual_hypo == self.current_state():
		
		self.print_stats()
		
	def add_hypo(self, trans, pos):
		h = Hypothesis(trans, pos)
		self.hypos_new.append(h)
		self.hypos.append( h )
		
	def current_pose(self):
		return self.current_state().pose()
		
	def current_state(self):
		return self.states[0]
		
	def print_stats(self):
		print "Context: Statistics"
		print "number of actions: ", len(self.actions_all)
		print "number of hypos:   ", len(self.hypos)
		print "number of states:  ", len(self.states)
		print "number of features:", len(self.features)
		print ""
		print "best hypo.: ", self.hypos[0].p()
		print "worst hypo.: ", self.hypos[len(self.hypos)-1].p()


robot = Robot(1,1,1)
ctxt = Context(robot)
