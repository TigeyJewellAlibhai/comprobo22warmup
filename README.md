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
