
---
### Writeup / README

#### 1. Provide a Writeup / README that includes all the rubric points and how you addressed each one.  You can submit your writeup as markdown or pdf.  

You're reading it! Below I describe how I addressed each rubric point and where in my code each point is handled.

### Explain the Starter Code

#### 1. Explain the functionality of what's provided in `motion_planning.py` and `planning_utils.py`
These scripts contain a basic planning implementation that includes.

### Implementing Your Path Planning Algorithm

#### 1. Set your global home position
Here i read the first line of the csv file, extract lat0 and lon0 as floating point values and use the self.set_home_position() method to set global home. 
I used csv library to read 1st line of csv file and assinged values into lat0, lon0 variables. Finally feed these into self.set_home_position() method to set global home.


#### 2. Set your current local position
Here as long as you successfully determine your local position relative to global home you'll be all set. 
I used the global_to_local method from planning_utils, which takes the self.global_position and elf.global_home as parameter and gives the current local position w.r.t NED co-ordinates.


#### 3. Set grid start position from local position
This is another step in adding flexibility to the start location. As long as it works you're good to go!
Here i defined the start_position of grid by using local NED co-ordinates values with map offset values.

#### 4. Set grid goal position from geodetic coords
This step is to add flexibility to the desired goal location. Should be able to choose any (lat, lon) within the map and have it rendered to a goal location on the grid.
I randomly took a point from simulator as goal_lon = -122.398709 goal_lat = 37.7936873. Then converted this into NED(local) co-ordinate values using global_to_local method. Finally defined converted NED co-ordinates w.r.t Map coordinates using offsets.

#### 5. Modify A* to include diagonal motion (or replace A* altogether)
Minimal requirement here is to modify the code in planning_utils() to update the A* implementation to include diagonal motions on the grid that have a cost of sqrt(2), but more creative solutions are welcome.

    NORTH_WEST = (-1, -1, np.sqrt(2))
    NORTH_EAST = (-1, 1, np.sqrt(2))
    SOUTH_WEST = (1, -1, np.sqrt(2))
    SOUTH_EAST = (1, 1, np.sqrt(2))
These 4 additional motions with cost of sqrt(2) added to previous Action set. Depending these actions adjusted few lines of code in the logic.

#### 6. Cull waypoints 
For this step you can use a collinearity test or ray tracing method like Bresenham. The idea is simply to prune your path of unnecessary waypoints. Explain the code you used to accomplish this step.
To accomplish this step i used collinearity test approach. For this i loaded all the path points into prune_path() function. The Prune path functions elements the values which area of 3 points less than 1e-6. Likewise performs for all the combitions of 3 points, finally produce pruned points. These points are loaded into waypoint varibale by converting these into NED frame values using offset. 
These waypoints will be call one by one and send into cmd_position function to mavigate the drone.



### Execute the flight
#### 1. Does it work?
It works!

### Double check that you've met specifications for each of the [rubric](https://review.udacity.com/#!/rubrics/1534/view) points.
  
# Extra Challenges: Real World Planning

For an extra challenge, consider implementing some of the techniques described in the "Real World Planning" lesson. You could try implementing a vehicle model to take dynamic constraints into account, or implement a replanning method to invoke if you get off course or encounter unexpected obstacles.


