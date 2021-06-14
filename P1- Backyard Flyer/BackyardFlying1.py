import time
from enum import Enum
import numpy as np
from udacidrone import Drone
from udacidrone.connection import MavlinkConnection
from udacidrone.messaging import MsgID

class Phases(Enum):   #1 create phases class
	MANUAL = 0
	ARMING = 1
	TAKEOFF = 2
	WAYPOINT = 3
	LANDING = 4 
	DISARMING = 5


class UpAndDownFlyer(Drone): #2 create a UpAndDownFlyer class which inherit from Drone parent class
	#Initializer for UpAndDownFlyer class
	def __init__(self, connection):
	 super().__init__(connection)           #Initializer for Drone parent class 
	 
	 self.target_position = np.array([0.0, 0.0, 0.0])  #varible array for drone position 
	 self.in_mission = True                           #******* mission status logic varible(Based on this will control loginc)
	 
	 self.mission_trasition = False
  
	 self.altitude = 0

  

	 #Initial state
	 self.flight_phase = Phases.MANUAL       # Set initial state
	 
	 #register all your callbacks here      #These call backs are running parallely whenenver conditions inside it becomes true, that callback executes first
	 self.register_callback(MsgID.LOCAL_POSITION, self.local_position_callback)
	 self.register_callback(MsgID.LOCAL_VELOCITY, self.velocity_callback)
	 self.register_callback(MsgID.STATE, self.state_callback)  
	 # self.register_callback(MsgID.MISSION, self.mission_callback)  
	 
	def local_position_callback(self): #8  This executes throgh rigister callbacks
		if self.flight_phase == Phases.TAKEOFF:
			# coordinate conversion 
			self.altitude = -1.0 * self.local_position[2]
			self.mission_trasition = True

			if self.altitude > 0.95 * self.target_position[2]:
				self.set_home_position(self.global_position[0], self.global_position[1], self.global_position[2])
				self.target_position = self.callWaypoint()            #Waypoints updated as target points
		
  
		elif self.flight_phase == Phases.WAYPOINT:
			# print(self.local_position[0], self.local_position[1])
			if len(self.target_position) > 0:      #check the target list size
				check = True
				pose = self.target_position[0]
				print('target_pose', pose)
				self.cmd_position(pose[0], pose[1], pose[2], 0)
				if np.linalg.norm(pose[0:2] - self.local_position[0:2]) < 1.0 and check:        #calcualtion using norm function
					self.target_position.pop(0)                      #remove first index list
					check = False
			
			else:
				print("Target square mission completed")
				self.landing_transition()

				
	def velocity_callback(self): #10
		if self.flight_phase == Phases.LANDING:
			if ((self.global_position[2] - self.global_home[2] < 0.1) and abs(self.local_position[2]) < 0.01):
				self.disarming_transition()
    
	def callWaypoint(self):
		target_points = [[15, 0, 3], [15, 10, 3], [0, 10, 3], [0, 0, 3]]            #Waypoints
		self.flight_phase = Phases.WAYPOINT
		print("waypoint mode activated")
		return target_points	
	 
	 

	 #Important callback function
	def state_callback(self): #3
		if not self.in_mission:  #In first time, in_mission varible set as True (but here this condn become false). So this condition wil skip. In second time this will become False at end of manual mission, that time this condition retruns non and skips below statements, so this entire class will stop.
			return
		if self.flight_phase == Phases.MANUAL:  #4
			self.arming_transition()
		elif self.flight_phase == Phases.ARMING:  #6
			self.takeoff_transition()
		elif self.flight_phase == Phases.DISARMING:   #11
			self.manual_transition()
		
	def arming_transition(self):  #5
		print("armed transition")
		self.take_control()
		self.arm()
		
		# set the current location to be the home position
		self.set_home_position(self.global_position[0], self.global_position[1], self.global_position[2])
		self.flight_phase = Phases.ARMING
		
	def takeoff_transition(self):  #7
		print("takeoff transition")
		target_altitude = 3.0
		self.target_position[2] = target_altitude
		self.takeoff(target_altitude)
		self.flight_phase = Phases.TAKEOFF
		
	def landing_transition(self):  #9 
		print("landing transition")
		time.sleep(2)
		self.land()
		self.flight_phase = Phases.LANDING
		
	def disarming_transition(self): #11
		print("disarm transition")
		self.disarm()
		self.flight_phase = Phases.DISARMING
		
				
	def manual_transition(self):  #12
		print("manual transition")
		self.release_control()
		self.stop()
		self.in_mission = False        
	
	def start(self):
		self.start_log("Logs", "NavLog.txt")
		print("starting connection")
		super().start()
		self.stop_log()  
	 
	 
if __name__ == "__main__":
	conn = MavlinkConnection('tcp:127.0.0.1:5760', threaded=False, PX4=False)  #0
	drone = UpAndDownFlyer(conn)
	time.sleep(2)
	drone.start()