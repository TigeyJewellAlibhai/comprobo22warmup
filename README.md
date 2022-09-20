# comprobo22warmup
Lilo Heinrich and Tigey Jewell-Alibhai

## Drive Square Behaviour

The goal of this behaviour was to get the robot to drive in a square by having it drive forward, turn, and repeat 3 more times. It could continue to do this indefinitely, but we decided it should stop after completing a full square.

To do this, we developed 3 modes, which could be switched by changing a variable: drive forward, turn, and stop. To make the robot drive a square, we alternated between driving forward and turning for 2 seconds each, using the in built timer and a couple of counter variables. After 8 switches total, we had the robot stop, thereby completing the square. We tuned the robot to turn 90 degrees by tuning the angular velocity and keeping the turning time constant.

The most difficult part of this behaviour was tuning the turn angle to 90 degrees, since this was hard coded into the angular velocity variable. Any slight difference from 90 degrees would compound over the 4 turns, making it more obvious. We ended up getting very close to 90.

## Wall Follow Behaviour

The goal of this behaviour was to get the robot to 'follow' a wall by detecting it and driving parallel to it. This can best be done using the LIDAR sensor.

We thought of several possible ways to do this, but we settled for one of the easier options involving detecting a wall on the right side of the robot. The robot would then scan angles 10 degrees behind and in front of its right side (270 degrees). If the 10 degrees in front were on average closer, it was oriented more towards the wall, and would steer left (away from the wall). If the 10 degrees behind the robot were on average closer, the robot was pointed away from the wall, and would steer right (towards the wall). The steering was proportional to the difference between the distance to the wall behind and in front. This worked surprisingly well on straight walls for such a simple algorithm.

We used a real neato (instead of gazebo) for testing this node, and the room where we initially tested it had black baseboards. The lidar could not detect the walls (presumably from the high light absorption of the black walls) so we had to test out in the hallway with lighter color walls.

## Person Follow Behaviour

The goal of this behaviour was to get the robot to 'follow' a person. The approach we took was to search for the a cluster of points nearest the robot. We decided on the simple approach of detecting the nearest cluster within a given set of parameters. This means our neato needs to be placed in a fairly open space without too many other obstacles in order to work. 

The clustering algorithm iterates from -180 to 180 degrees, looking at the difference between the range of the previous and current point. If the difference is greater than a threshold, it does not belong to the cluster. The code can skip one outlier and continue the cluster, but more than one outlier in a row causes it to end the current cluster and create a new one. We filter on number of points and sort on average range to find the closest, this is the "person" to follow. By calibrating the number of points filter, it no longer drives towards table and chair legs. 

To drive, the angular velocity component uses a linear multiplier on the difference between the heading of the "person" and the heading of the neato currently. For linear speed, again we mutliply the distance between the person and the neato, but also divide by the square root of the difference in heading to ensure the neato doesn't zoom off in the wrong direction. We started by simulating in gazebo, then made changes to the code and calibrated coefficients while running on a real neato. There was data noise in the real world to account for, and the neato could see a whiteboard more clearly than our feet. It successfully follows us accross the room even through tables and chairs, but would get stuck at walls. 

In the future there would be potential to use smarter algorithms for clustering/making sense of points it sees, and we could try filtering on only moving points or keeping track of where the "person" previously was in the odom frame.

## Object Avoidance

The goal of this behaviour was to move forward while reactively avoiding obstacles in its path. 

The approach we took was to have the neato find clusters just like in the person follower, reporting the average angle and range of each cluster. The neato then finds the closest cluster by finding the cluster with the minimum average range within an average angle range of -50 to 50. This closest cluster is what the neato plans to avoid. If the closest obstacle is more than a meter away, the neato decides to drive at a constant speed toward the heading of 0 degrees in the odometry frame, so it remembers which way is "forward". If the cluster average range is within 1 meter, the neato makes corrective action to turn away from the object, turning more or less based on the angle of the cluster in relation to the neato.

We tested this code mainly in gazebo and it successfully weaves through 4 obstacles of cylinders and cubes. Some possible failure modes of this algorithm include too closely spaced objects, and scenarios when the neato must recognize and avoid more than one object at a time. 
