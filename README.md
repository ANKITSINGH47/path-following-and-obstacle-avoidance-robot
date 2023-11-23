# Path Planning with Obstacle Avoidance

## 1 PROBLEM STATEMENT
The aim of the experiment is to make the robot move from the
point (x,y) = (2,4) to the point (x,y) = (10,4). The robot has to
pass through the point (x,y) = (2,10) on its way to the destination.
Further, the robot has to avoid obstacles on its way as it moves
through the target goals.

## 2 PURE PURSUIT ALGORITHM
Pure pursuit is a tracking algorithm that works by calculating
the curvature that will move a vehicle from its current position
to some goal position. In this algorithm we choose some waypoints that is ahead of our path. Our vehicle could be thought
of as chasing these points. The name is basically derived from how humans drive the vehicle. We look ahead the road and
decide accordingly to change the direction of the vehicle. This
point (look-ahead) changes every time we move forward.

## 2.1 GEOMETRIC APPROACH OF PURE PURSUIT
### ALGORITHM
The pure pursuit approach is a method of geometrically
determining the curvature that will drive the vehicle to a chosen
way point.This goal point is a point on the path that is one look
ahead distance from the current vehicle position. An arc that
joins the current point and the goal point is constructed. The
chord length of this arc is the look ahead distance, and acts as
the third constraint in determining a unique arc that joins the
two points.
Let point (x, y) be a way-point , which is a look ahead distance
from the origin. The curvature of the arc that joins the origin
to point (x, y) and whose chord length is l is calculated as

### 2.2 INPUTS TO ALGORITHM
1. Linear velocity: This is specified as a scalar in meters per
second.It is assumed that the vehicle drives at a constant linear
velocity and that the computed angular velocity is independent
of the linear velocity.
2. Angular velocity: Maximum angular velocity, specified a scalar
in radians per second. The controller saturates the absolute
angular velocity output at the given value.
3. Look-ahead distance: The look-ahead distance is how far along
the path the robot should look from the current location to
compute the angular velocity commands.
4. Vector of ordered waypoints(path):Waypoints, specified as an
n-by-2 array of [x y] pairs, where n is the number of waypoints.

### 2.3 OUTPUTS OF ALGORITHM
1. Linear velocity: scalar in meters per second
2. Angular velocity: specified as a scalar in radians per second

### 2.4 ALGORITHM
1. Determine current location
2. Find path points closest to vehicle
3. Find the goal point (x, y) on the path at the distance ′l′
from the current location
4. Transform the goal point to robot frame: The geometric
derivation for the curvature was done in vehicle coordinates
and curvature commands to the vehicle make sense in vehicle
coordinates
5. Find the curvature of such a curve C such that the ′l′
is the chord length of C
6. Set steering to the differential drive robot along a given
curvature

### 2.5 USING SIMULINK PURE PURSUIT BLOCK
In order to understand the basic working of pure pursuit
algorithm, we first used the pure pursuit block available in
simulink for path planning as shown in below figure



## 3 OBSTACLE AVOIDANCE USING VECTOR
FIELD HISTOGRAM
This method basically detects the unknown obstacles and
avoids collisions while simultaneously steering the mobile robot
toward the target. It uses a two-dimensional Cartesian
histogram grid as a world model. The VFH method employs
a two-stage data reduction technique and three levels of data
representation exist:
1. The highest level holds the description of the robot’s
environment.The two-dimensional Cartesian histogram
grid C is continuously updated in real-time with range data
sampled by the on-board range sensors.
2. At the intermediate level, a 1 − D polar histogram H is
constructed around the robot’s momentary location. H
comprises n angular sectors of width a. This maps the
active region C∗ onto H, resulting in each sector k holding
a value hk that represents the polar obstacle density in the
direction that corresponds to sector k.
3. The lowest level of data representation is the output of the
VFH algorithm: the reference values for the drive and steer
controllers of the vehicle.

### 3.1 INPUTS TO ALGORITHM
1. Ranges: Range values from scan data, specified as a vector
in meters. These range values are distances from a sensor
at given angles. The vector must be the same length as the
corresponding angles vector.
2. Angles: Angle values from scan data, specified as a vector
in radians. These angle values are the specific angles of the
given ranges. The vector must be the same length as the
corresponding ranges vector.
3. Target Direction: Target direction for the vehicle, specified
as a scalar in radians. The forward direction of the vehicle
is considered zero radians, with positive angles measured
counterclockwise.

### 3.2 OUTPUT OF ALGORITHM
1. Steer Direction: Steering direction for the vehicle, specified
as a scalar in radians.The forward direction of the vehicle
is considered zero radians, with positive angles measured
counterclockwise.

### 3.3 ALGORITHM
1. Direction of cell is calculated
βij = tan−1
yj − y0
xi − x0
where (x0, y0) is the vehicle centre and (xj
, yj ) is the active cell
coordinate.
2. Cost function: mij = (cij)
2

a − b
p
(xi − x0)
2 + (yi − y0)
2

where a and b parametrs need to be tuned. mij is
proportional to −d. Therefore, occupied cells produce large
vector magnitudes when they are in the immediate vicinity
of the robot, and smaller ones when they are further away.
Specifically, a and b are chosen such that a − bdmax = 0, where
dmax =
√
2(w −1)/2 is the distance between the farthest active
cell and the Vehicle Center Point(VCP). This way m = 0 for the
farthest i, j active cell and increases linearly for closer cells. C
∗
is squared. This expresses the confidence that recurring range
readings represent actual i, j obstacles, as opposed to single
occurrences of range readings, which may be caused by noise.
3. Create angular histogram H with resolution such that n =
180
α
.
4. A threshold (user specified) is used to detect the candidate
valley: Any valley comprised of sectors with smoothed polar
obstacle density (POD)s below a certain threshold is called a
candidate valley.
5. Choose the candidate valley aligned most with target.
6. Steers through the middle of the chosen valley: First, the
algorithm measures the size of the selected valley (i.e., the
number of consecutive sectors with PODs below the threshold).
Here, two types of valleys are distinguished, namely,wide and
narrow ones. A valley is considered wide if more than smax
consecutive sectors fall below the threshold. Wide valleys result
from wide gaps between obstacles or from situations where only
one obstacle is near the vehicle. The sector that is nearest to
ktarg and below the threshold is denoted kn and represents the
near border of the valley. The far border is denoted as kf and
is defined as kf = kn + smax. The desired steering direction q
is then defined as q =
kn+kf
2
. where,
a, b Positive constants
C
∗
i,j Certainty value of active cell (i, j)
di,j Distance between active cell (i, j) and the VCP
mi,j Magnitude of the obstacle vector at cell (i, j)
x0, y0 Present coordinates of the VCP
xi
, yj Coordinates of active cell (i, j)
βi,j Direction from active cell (i, j) to the VCP

### 3.4 USING SIMULINK VECTOR FIELD HISTOGRAM
BLOCK
In order to understand the basic working of vector field
histogram, we first used the VHF block available in simulink
for obstacle avoidance as shown in below figure


### 3.5 IMPLEMENTATION OF VECTOR FIELD HISTOGRAM
In this section, we implement the vector field histogram from scratch
using the user defined function block in simulink


## 4 RESULTS & OBSERVATIONS

<img width="250" alt="path following avoiding obstacle" src="https://github.com/ANKITSINGH47/path-following-and-obstacle-avoidance-robot/assets/47277960/c1c1da18-e567-481d-a0d6-9a68c172c6c4">\n
























<img width="250" alt="path-track2" src="https://github.com/ANKITSINGH47/path-following-and-obstacle-avoidance-robot/assets/47277960/be3cff71-22ee-4727-b7ce-144b8a878781">


