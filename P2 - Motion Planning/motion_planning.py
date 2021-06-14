import argparse
import time
import msgpack
from enum import Enum, auto

import numpy as np
import time
from planning_utils import a_star, heuristic, create_grid, global_to_local, prune_path
from udacidrone import Drone
from udacidrone.connection import MavlinkConnection
from udacidrone.messaging import MsgID
import csv
import matplotlib.pyplot as plt

class States(Enum):
    MANUAL = auto()
    ARMING = auto()
    TAKEOFF = auto()
    WAYPOINT = auto()
    LANDING = auto()
    DISARMING = auto()
    PLANNING = auto()           #Extra state added compare to Backyard project


class MotionPlanning(Drone):

    def __init__(self, connection):
        super().__init__(connection)

        self.target_position = np.array([0.0, 0.0, 0.0])
        self.waypoints = []
        self.in_mission = True
        self.check_state = {}

        # initial state
        self.flight_state = States.MANUAL

        # register all your callbacks here
        self.register_callback(MsgID.LOCAL_POSITION, self.local_position_callback)
        self.register_callback(MsgID.LOCAL_VELOCITY, self.velocity_callback)
        self.register_callback(MsgID.STATE, self.state_callback)

    def local_position_callback(self):
        if self.flight_state == States.TAKEOFF:
            if -1.0 * self.local_position[2] > 0.95 * self.target_position[2]:
                self.waypoint_transition()
        elif self.flight_state == States.WAYPOINT:
            if np.linalg.norm(self.target_position[0:2] - self.local_position[0:2]) < 1.0:
                if len(self.waypoints) > 0:
                    self.waypoint_transition()
                else:
                    if np.linalg.norm(self.local_velocity[0:2]) < 1.0:
                        self.landing_transition()

    def velocity_callback(self):
        if self.flight_state == States.LANDING:
            if self.global_position[2] - self.global_home[2] < 0.1:
                if abs(self.local_position[2]) < 0.01:
                    self.disarming_transition()

    def state_callback(self):
        if self.in_mission:
            if self.flight_state == States.MANUAL:
                self.arming_transition()
            elif self.flight_state == States.ARMING:
                if self.armed:
                    self.plan_path()
            elif self.flight_state == States.PLANNING:
                self.takeoff_transition()
            elif self.flight_state == States.DISARMING:
                if ~self.armed & ~self.guided:
                    self.manual_transition()


    def arming_transition(self):
        self.flight_state = States.ARMING
        print("arming transition")
        self.arm()
        self.take_control()

    def takeoff_transition(self):
        self.flight_state = States.TAKEOFF
        print("takeoff transition")
        print("takeoff to", self.target_position[2])
        self.takeoff(self.target_position[2])

    def waypoint_transition(self):
        self.flight_state = States.WAYPOINT
        print("waypoint transition")
        self.target_position = self.waypoints.pop(0)
        print('target position', self.target_position)
        self.cmd_position(self.target_position[0], self.target_position[1], self.target_position[2], self.target_position[3])
        # print("current pose", self.local_position)

    def landing_transition(self):
        self.flight_state = States.LANDING
        print("landing transition")
        self.land()

    def disarming_transition(self):
        self.flight_state = States.DISARMING
        print("disarm transition")
        self.disarm()
        self.release_control()

    def manual_transition(self):
        self.flight_state = States.MANUAL
        print("manual transition")
        self.stop()
        self.in_mission = False

    def send_waypoints(self):
        print("Sending waypoints to simulator ...")
        data = msgpack.dumps(self.waypoints)
        self.connection._master.write(data)

    def plan_path(self):
        self.flight_state = States.PLANNING
        print("Searching for a path ...")
        TARGET_ALTITUDE = 5
        SAFETY_DISTANCE = 5

        self.target_position[2] = TARGET_ALTITUDE

        # TODO: read lat0, lon0 from colliders into floating point values
        filename = '/home/darshan/udacity/Motion planning/6.Project/FCND-Motion-Planning/colliders.csv'
        with open(filename) as f:
            reader = csv.reader(f)
            row1 = next(reader) 

        lon , lat = row1[0], row1[1]
        lon0, lat0 = float(lon[5:]), float(lat[6:])
        print(lon0, lat0)
        
        # TODO: set home position to (lon0, lat0, 0)
        self.set_home_position(lon0, lat0, 0)         #It will be in lon, lat, alt format(Geodict format)
        # self.set_home_position( -122.397477, 37.792691,  0)         #It will be in lon, lat, alt format(Geodict format)

        # TODO: retrieve current global position
        drone_global_position = self.global_position    
        print("drone global", drone_global_position)
 
        # TODO: convert to current local position using global_to_local()     #Refer the description given in planning utils code to understand better on global, home and local positions
        drone_local_position = global_to_local(drone_global_position, self.global_home)         #It will be in North, East, Down(NED) format
        print("drone_local_position", drone_local_position)
        
        
        print('global home {0}, position {1}, local position {2}'.format(self.global_home, self.global_position,
                                                                         self.local_position))
        # Read in obstacle map
        data = np.loadtxt('/home/darshan/udacity/Motion planning/6.Project/FCND-Motion-Planning/colliders.csv', delimiter=',', dtype='Float64', skiprows=2)
        
        # Define a grid for a particular altitude and safety margin around obstacles
        grid, north_offset, east_offset = create_grid(data, TARGET_ALTITUDE, SAFETY_DISTANCE)   #This creates the grid 2D configuration(map) and also returns the origin of left downmost obstacle grid pose from mid obstacle grid pose of map 
        print("North offset = {0}, east offset = {1}".format(north_offset, east_offset))        #ie, map size is 921,921 and mid map is 0,0 (but w.r.t obstacle info, map center is 316, 445 is map center also -316, -445 left downmost obsmap end and 632, 850 right upmost obs map end)
                                                                                                # So north_offset = 316, east_offset = 445     
              
              
        # TODO: convert start position to current position rather than map center      (NED Frame)
        grid_start = (int(drone_local_position[0] - north_offset) , int(drone_local_position[1] - east_offset))               #Here, grid_start = (0 - (-316)), (0 -(-445)) = 316,445 is grid map center 
        print("grid_start", grid_start)
            
        
        
        # Set goal as some arbitrary position on in global (GPS) coordinates
        # TODO: adapt to set goal as latitude / longitude position and convert
        goal_lon = -122.398709
        goal_lat = 37.7936873
        global_goal = (goal_lon, goal_lat, 0)
    
        # Convert to local (NED) coordinates
        local_goal = global_to_local(global_goal, self.global_home)
        
        # Convert to grid coordinates
        grid_goal = (int(local_goal[0]) - north_offset,
                     int(local_goal[1]) - east_offset)



        # Run A* to find a path from start to goal
        # TODO: add diagonal motions with a cost of sqrt(2) to your A* implementation (This ToDo added in planning utils.py script)
        
        print('Local Start and Goal: ', grid_start, grid_goal)
        print('Planning, please wait...')
        t1 = time.time()
        path, _ = a_star(grid, heuristic, grid_start, grid_goal)
        print('Planning took {} seconds'.format(time.time() - t1))
        print('Number of waypoints: {}'.format(len(path)))

        # print(path)
        
        
        # TODO: prune path to minimize number of waypoints
        print('Pruning starts, please wait...')
        t1 = time.time()
        pruned_path = prune_path(path)
        print('Planning took {} seconds'.format(time.time() - t1))
        print('Number of waypoints: {}'.format(len(pruned_path)))
        print(pruned_path)
        
        
        # Convert path to waypoints 
        waypoints = []
        for i in range(len(pruned_path)):
            px = pruned_path[i][0] + north_offset
            py = pruned_path[i][1] + east_offset
            pz = TARGET_ALTITUDE

            px_prev = pruned_path[max(i-1, 0)][0] + north_offset
            py_prev = pruned_path[max(i-1, 0)][1] + east_offset

            yaw = np.arctan2(py - py_prev, px - px_prev)         #Equation for calculating yaw between 2 unknown points (https://stackoverflow.com/questions/18184848/calculate-pitch-and-yaw-between-two-unknown-points)
            waypoints.append([px, py, pz, yaw])
            
            
            #  OR
        # waypoints = [[p[0] + north_offset, p[1] + east_offset, TARGET_ALTITUDE, 0] for p in pruned_path]  
        
        
        # Set self.waypoints
        self.waypoints = waypoints
        print("Waypoints", self.waypoints)
        
        # TODO: send waypoints to sim (this is just for visualization of waypoints)
        
        self.send_waypoints()

    def start(self):
        self.start_log("Logs", "NavLog.txt")

        print("starting connection")
        self.connection.start()

        # Only required if they do threaded
        # while self.in_mission:
        #    pass

        self.stop_log()


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument('--port', type=int, default=5760, help='Port number')
    parser.add_argument('--host', type=str, default='127.0.0.1', help="host address, i.e. '127.0.0.1'")
    args = parser.parse_args()

    conn = MavlinkConnection('tcp:{0}:{1}'.format(args.host, args.port), timeout=1200)
    drone = MotionPlanning(conn)
    time.sleep(1)

    drone.start()
