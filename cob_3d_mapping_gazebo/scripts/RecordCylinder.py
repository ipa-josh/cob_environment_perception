#!/usr/bin/python
import sys
import roslib
roslib.load_manifest('cob_3d_mapping_gazebo')
roslib.load_manifest('cob_script_server')

import rospy
import os
import math
import time

from gazebo.srv import *
from simple_script_server import *

import getopt
import tf

class broadcaster (object):

    def __init__(self,in_frame,out_frame):
        self.br = tf.TransformBroadcaster()
        self.in_frame= in_frame
        self.out_frame = out_frame
        
    def set_tf(self,x,y,alpha):
        self.x = x
        self.y = y
        self.alpha = alpha
        
    def Run(self):
        self.br.sendTransform((self.x,self.y, 0),
                    tf.transformations.quaternion_from_euler(0, 0, self.alpha),
                    rospy.Time.now(),self.out_frame,self.in_frame) 
                    
     

##
# Class to record cylinder
class RecordCylinderScript(script):

    def __init__(self,num_steps,intervall):

        rospy.init_node("recorder")
        self.sss = simple_script_server()
        
        #assign members        

        self.do_tf = False
        self.do_verbose= True
        
        self.intervall = intervall
        
        self.num_steps = num_steps
        
        self.srv_set_model_state = rospy.ServiceProxy('/gazebo/set_model_state', SetModelState)
        self.req_set = SetModelStateRequest()
        self.req_set.model_state.model_name = "robot"
        self.req_set.model_state.reference_frame = "map"
        
      

        
        
        if do_tf ==True:
            self.tf_br = broadcaster("/map","/odom_combined",)

        
        



    def set_flags(self,do_tf,do_verbose):
        self.do_tf = do_tf

        self.do_verbose = do_verbose
    
    def set_init_pose(self):
        self.sss.move("arm","folded")
        self.sss.move("tray","down") 

        self.req_set.model_state.pose.position.x = -1.5
        self.req_set.model_state.pose.position.y = -1.5
        self.req_set.model_state.pose.position.z = 0
        self.req_set.model_state.pose.orientation.w = 1
        self.req_set.model_state.pose.orientation.x = 0
        self.req_set.model_state.pose.orientation.y = 0
        self.req_set.model_state.pose.orientation.z = 0            
        self.res_set = self.srv_set_model_state(self.req_set)  

            
            


    def RunCircle(self, center,radius):

        # output        
        print "[RecordCylinderScript]--> Circle Paramters:"
        print "                          Center = ( %f , % f )" % (center[0],center[1])
        print "                          Radius = %f " % radius
        print "                          Steps  = %i " % (self.num_steps)
        # cob kitchen warnings
        if radius > 1:
            print "WARNING DANGEORUS COB_KITCHEN PARAMETER (use radius <= 1)"
            raw_input("to continue anyway pres [Enter] to abort [CTRL+C]") 
    
        trajectory = self.calc_circle(center,radius)
        self.Move(trajectory)
        
    def RunLine(self,start,end,ori):
        print "[RecordCylinderScript]--> Line Paramters:"
        print "                          Start = ( %f , % f )" % (start[0],start[1])
        print "                          End = ( %f , % f ) " % (end[0],end[1])
        print "                          Ori = %f" % ori
        print "                          Steps  = %i " % (self.num_steps)
    
        trajectory = self.calc_line(start,end,ori)
        self.Move(trajectory)    
        
        
    def RunSingle(self,pos,ori):
        print "[SinglePose]--> Line Paramters:"
        print "                          Pos = ( %f , % f )" % (pos[0],pos[1])
        print "                          Ori = %f " % (ori)
        
        trajectory =self.calc_single(pos,ori)
        
        self.Move(trajectory)


    def calc_single(self,pos,ori):
        
        trajectory = list()
        positions_x=list()
        positions_y=list()
        alpha      =list()
        positions_x.append(pos[0])
        positions_y.append(pos[1])
        alpha.append(ori)
        
        trajectory.append(positions_x)
        trajectory.append(positions_y)
        trajectory.append(alpha)
        return trajectory
            
    def calc_line(self,start,end,ori):
        positions_x=list()
        positions_y=list()
        alpha      =list()
        step = 0
        PI = math.pi
        phi = 0
        
        direction_x = end[0]-start[0]
        direction_y = end[1]-start[1]

        while step < self.num_steps:
                d_inc = 1*step /self.num_steps
                     
                positions_x.append(start[0]+ d_inc * direction_x  )
                positions_y.append( start[1]+ d_inc * direction_y  )
                alpha.append(ori)
                step = step+1 
            
            
        trajectory = list()
        trajectory.append(positions_x)
        trajectory.append(positions_y)
        trajectory.append(alpha)
        print "[RecordCylinderScript]--> Line trajectory calculated"
        return trajectory
                 
                

        
            
                
         
            
    def calc_circle(self,center,radius):
 
        
        
        #create circular positions with orientatios
        positions_x=list()
        positions_y=list()
        alpha      =list()
        step = 0
        PI = math.pi
        phi = 0
        delta_phi = 2*PI / self.num_steps
        while step < self.num_steps:
            phi = (step)*delta_phi      
            positions_x.append( center[0] + radius * math.cos(phi) )
            positions_y.append( center[1] +radius * math.sin(phi) )
            
           # if phi > PI:
           #     alpha.append( phi -2*PI )
           # else:
            alpha.append(phi)
                                 
            step =step+1
        trajectory = list()
            
        trajectory.append(positions_x)
        trajectory.append(positions_y)
        trajectory.append(alpha)
        print "[RecordCylinderScript]--> Trajectory calculated" 
        return trajectory
        
        
        
    
    def Move(self,trajectory):
    
    
    
        positions_x = trajectory[0]
        positions_y = trajectory[1]
        alpha       = trajectory[2]
        

        step = 0                
        for a in positions_x:   
            if self.do_tf == True:
                self.tf_br.set_tf(positions_x[step],positions_y[step],alpha[step])
                self.tf_br.Run()
            
            # calculate quaternion
            
            q = tf.transformations.quaternion_from_euler(0,0,alpha[step])
            
            #set model_state to current pose on circle        
            self.req_set.model_state.pose.position.x = positions_x[step]
            self.req_set.model_state.pose.position.y = positions_y[step]
            self.req_set.model_state.pose.position.z = 0
            self.req_set.model_state.pose.orientation.w = q[3]
            self.req_set.model_state.pose.orientation.y = 0
            self.req_set.model_state.pose.orientation.z = q[2]
            self.res_set = self.srv_set_model_state(self.req_set)
            
            

            
          
            #increment step and let sleep for 1 second    

            print "[RecordCylinderScript]--> Assumed position % i  of  % i  " % (step,self.num_steps)
            step = step+1
            time.sleep(self.intervall)
            
    def PitchHead(self,command):
        if command == "up":
            pos = [[-0.2,0,-0.2]]
        elif command == "down":        
            pos =  [[0.1,0,0.1]]
        else:
            rospy.logwarn('pitch direction not valid use [up] and [down] - ptiching to [0 0 0]')

            pos = [[0,0,0]]                 

        move_handle = self.sss.move("torso",pos)
        time.sleep(6)


        
        
        
            
    def Spawn(self):
        #spawn cylinder
        print "[RecordCylinderScript]--> Set robot to initial pose"
        self.set_init_pose()
        if do_verbose == False:
            print "[RecordCylinderScript]--> No Verbose Output"
            os.system("roslaunch cob_3d_mapping_gazebo spawn_cylinder.launch >/dev/null")
        else:
            os.system("roslaunch cob_3d_mapping_gazebo spawn_cylinder.launch ")
        print "[RecordCylinderScript]--> Spawning Cylinder" 
        
        

        
                
        
if __name__ == "__main__":
    #set flags to default values
    do_tf = False

    do_verbose = True
    
    run_circle = True
    run_line = False
    run_single = False
    run_pitch = False
    run_spawn = False
    run_helix = False
    
    
    intervall = 0.3
    num_steps = 12
    radius = 1
    center = (0,0)
    # parse command line options
    try:
        opts, args = getopt.getopt(sys.argv[1:], "h", ["help"])
    except getopt.error, msg:
        print msg
        print "for help use --help"
        sys.exit(2)
    # process options
    for o, a in opts:
        if o in ("-h", "--help"):
            print "\n\nUsage:\t rosrun RecordCylinder.py <mode> <options>"
            print " -- when run without <options> argument a default circle trajectory is used\n"
            
            print"<mode>   \t spawn .......... spawn default cylinder"
            print"          \t single [x0][y0][orientation] .. set parameters of single pose"
            print"          \t line [start][end][orientation] .. set parameters of trajectory line"
            print"          \t circle [x0][y0][r] .. set parameters of trajectory circle"
            print"          \t helix [x0][y0][r] .. set parameters of helix circle - looks up, then middle, then down"
            print"          \t pitch [up] | [down] ....... pitch torso up or down "
            print"Note: Only one mode can be chosen at a time. "
            print"<options> \t V .............. disable verbose output of spawned cylinder"
            print"          \t tf ............. publish transform in frame /map"
            print"          \t I [time]........ set rest intervall at every step to time"
            print"          \t N [#] .......... number of steps on trajectory"
            print"Note: multiple options can be used at the same time by just adding the to the command line"

            print"-------------------------------------------------------------------------"
            print"default values\t V = True , tf = False , I 0.3 , cirlce 0 0 1 , N 12 "   
            print"Usage example: RecordCylinder.py circle 0 0 1 N 100 I 0.1 tf"
            sys.exit(0)

            
    # process arguments

    for i in range(len(args)):

        arg = args[i]
        if (arg in 'V')  == True:
            print "no verbose activated"
            do_verbose = False
            continue        
        elif (arg in 'spawn')  == True:
            print "spawning default cylinder"
            run_circle = False
            run_spawn = True                        
            continue
        elif (arg in "tf")  == True:
            print "tf = true"
            do_tf = True
            continue
        elif(arg in "I") == True:
            print ("intervall set to %s" % args[i+1])
            intervall = (float)(args[i+1])
            continue
        elif(arg in "N") == True:
            print ("number of steps set to %s" % args[i+1] )
            num_steps = (float)(args[i+1])         
            continue              
        elif(arg in "circle") == True:
            print ("circle paramaters set")            
            center = ((float)(args[i+1]),(float)(args[i+2]))            
            radius = (float)(args[i+3]) 
       
            continue
        elif(arg in "line") == True:
            print ("line paramaters set")
            run_line = True
            run_circle = False
            start = ((float)(args[i+1]),(float)(args[i+2]))          
            end = ((float)(args[i+3]),(float)(args[i+4]))
            ori = (float)(args[i+5])
            continue

        elif(arg in "single") == True:
            print ("single pose paramaters set")

            run_circle = False
            run_single = True
            pos = ((float)(args[i+1]),(float)(args[i+2]))          
            ori = (float)(args[i+3])
            continue
        elif(arg in "pitch") == True:

            run_circle = False

            run_pitch = True
            command = (args[i+1])                
            continue
            
#        combinations of trajectories
        elif(arg in "helix") == True:
            run_circle = False
            run_helix  = True
            continue
            

        
                
            
            
                
    #parameters for trajectory


    
    #initialize script

    SCRIPT = RecordCylinderScript(num_steps,intervall)
    SCRIPT.set_flags(do_tf,do_verbose)
  
    
    #run script
    if run_line == True:
    
        SCRIPT.RunLine(start,end,ori)
    elif run_circle == True:
        SCRIPT.RunCircle(center,radius)
    elif run_single == True:
        SCRIPT.RunSingle(pos,ori)
    elif run_pitch == True:
        SCRIPT.PitchHead(command)
    elif run_spawn == True:
        SCRIPT.Spawn()  
    elif run_helix == True:
        SCRIPT.PitchHead("up")
        SCRIPT.RunCircle(center,radius)
        SCRIPT.PitchHead("middle")
        SCRIPT.RunCircle(center,radius)
        SCRIPT.PitchHead("down")
        SCRIPT.RunCircle(center,radius)
