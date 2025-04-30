# Task 3: Path Planning

## Objectives 
Use Path Planner to create more complex autonomous routines for Romis.

## Resources 
- [Path Planner docs](https://pathplanner.dev/pplib-getting-started.html)
- [Video Overview](https://www.youtube.com/watch?v=todRXAK239g)

## Task Details -- Part 1
Instead of planning out every rotation and translation of your autos, you can get more complex movement with path planning! Start with installing Path Planner on your computer. 

Experiment with creating paths (formed via adjusting your control points) and assembling paths together to form autos. Besides Follow Path commands that add your paths to an auto, you can chain commands together in command groups like sequential groups (run one after another), parallel groups (run all at the same time), race groups (first command that ends will stop the entire parallel group), etc. Just remember that commands that depend on the same subsystem cannot be run in parallel. The command name specifies which named command (must be bound in the code) should be run. Read about named commands [here](https://pathplanner.dev/pplib-named-commands.html).

You can also link waypoints so the next path in an auto starts in the location where the previous path ended.
<p align="center">
  <img src="path_planner.png" width="600">
</p>


## Task Details -- Part 2

### Create named commands 
Using your existing commands, register the commands via the Named Commands class so Path Planner will be able to recognize them when configuring autos.

### Create paths and autos in Path Planner
#### Set the Path Planner project to your code project directory.
#### Create a drive forward path for calibration purposes. Add it to an auto.
#### Create a path with an intermediate waypoint and chain it with another path in an auto. 
Make sure to link start and end waypoints so the robot odometry doesn’t think it’s teleporting!
#### Create any other autos you want to test
If you check the deploy directory in your code project, the path and auto jsons should show up there.

### Set up Auto Builder
#### Install PathPlannerLib as a vendor dependency in your project
#### Configure path follower in drivetrain
For tank drivetrains like Romis, you will have to configure a Ramsete path follower in the drivetrain class’ constructor, which is a static method on Path Planner’s Auto Builder class. This is important so the path planner knows which Romi drivetrain methods to call to move the Romi accurately. You’ll also need to create a few extra methods so you have all the necessary parameters (since the method requires Suppliers as inputs, you need to pass in the methods via the method reference operator).
Note: create a Differential Drive Odometry object to get and reset robot pose
#### Use the Auto Builder to add your autos to the auto Sendable Chooser
#### Test your autos in autonomous mode using the simulator!
