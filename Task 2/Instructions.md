# Task 2: Basic Control Systems (PID) 

## Objectives
Implement PID to control the robot more precisely.
## Resources
- [WPILib PID controllers](https://docs.wpilib.org/en/stable/docs/software/advanced-controls/controllers/pidcontroller.html)
- [PID Simulator](https://www.grauonline.de/alexwww/ardumower/pid/pid.html)
- [FRC Wiki PID Controller](https://firstwiki.github.io/wiki/pid-controller)

## Task Details
Without any control system in place, it will lead to over or under compensating distance/rotation movement due to factors like friction and the inertia of the object. Therefore, we use control systems like PID (proportional integral derivative) to adjust the motion of the moving object at each timestep based on its distance from the setpoint (using the proportional term), accumulated error (using the integral term), and rate of change of distance from the setpoint (using the derivative term).

You must define a PID Controller object (one for rotation and one for translation) and set wheel speeds to the output of the PID controller, so the robot will be able to adaptively slow down as it approaches the setpoint. You will have to tune the gains of the PID controller to optimize its behavior. 

### Quiz: which PID controller output is best?
<p align="center">
  <img src="pid.png" width="400">
</p>


<details>
    <summary>
        Answer
    </summary>  
    The red output signal because it converges most quickly to the desired setpoint with the least oscillation. 
</details>  

## Implementation   
We will be modifying the project from task 1 to incorporate the PID control system to more efficiently and accurate control the  Romi's behavior.

First, we will initialize PID controllers in the drivetrain and then use their control output in our rotation and translation commands to get to setpoints more precisely, since the motion of the robot will slow as the error between the current position and the setpoint decreases. 

First, instantiate the PID controllers, initializing them in the constructor of RomiDrivetrain. Note that we are using different PID gains (the constants representing kP, kI, and kD) for the different controllers since they control different aspects of the robot's motion. Later, we will plot the current robot position and setpoint on the same graph to tune these gains to more efficiently drive the Romi to setpoint.
<details>
    <summary>
        <a href="Solution/src/main/java/frc/robot/subsystems/RomiDrivetrain.java#L38">Solution</a>
    </summary>

    private final PIDController rotController;
    private final PIDController translateController;

    public RomiDrivetrain() {
    ... 

    translateController = new PIDController(kTranslationP, kTranslationI, kTranslationD);
    rotController = new PIDController(kRotationP, kRotationI, kRotationD);
    translateController.setTolerance(DriveConstants.translationTolerance.in(Meters));
    rotController.setTolerance(DriveConstants.rotationTolerance.in(Rotations));
  }

</details>
<br>

Make sure to define rotation and translation tolerances in constants that represent the maximum error allowable to still be considered to be at setpoint. Then, in the constructor of Romi drivetrain, apply these tolerances to the PID controllers so we can use them to find out when the Romi have reached its setpoint.

<details>
    <summary>
        <a href="Solution/src/main/java/frc/robot/Constants.java#L25">Solution</a>
    </summary>

    public static final Distance kTranslationTolerance = Inches.of(1);
    public static final Angle kRotationTolerance = Rotations.of(0.05);

</details>
<br>

Now, let's define methods to calculate PID controller output based on a current position and a setpoint. Use the controller's calculate method to fetch the controller output.

<details>
    <summary>
        <a href="Solution/src/main/java/frc/robot/subsystems/RomiDrivetrain.java#L88">Solution</a>
    </summary>

    public double calculateRotOutput(Angle setpoint) {
        curRotSetpoint = setpoint;
        return rotController.calculate(getAngle().baseUnitMagnitude(), setpoint.baseUnitMagnitude());
    }

    public double calculateTranslateOutput(Distance setpoint) {
        curTransSetpoint = setpoint;
        return translateController.calculate(getAverageDistance().baseUnitMagnitude(), setpoint.baseUnitMagnitude());
    }

</details>
<br>

Also in the RomiDrivetrain class, create methods to return whether each controller has reached setpoint based on the output of their atSetpoint methods.
<details>
    <summary>
        <a href="Solution/src/main/java/frc/robot/subsystems/RomiDrivetrain.java#L96">Solution</a>
    </summary>

    public boolean atTranslationSetpoint() {
        return translateController.atSetpoint();
    }

    public boolean atRotationSetpoint() {
        return rotController.atSetpoint();
    }

</details>
<br>

In the translation command from the previous task, modify the execute method so the applied translation speed is the output of the PID controller given the drivetrain's current location and the desired distance traveled. End the command once setpoint is reached. 
<details>
    <summary>
        <a href="Solution/src/main/java/frc/robot/commands/TranslateCommand.java#L26">Solution</a>
    </summary>

    @Override
    public void execute() {
        drive.arcadeDrive(drive.calculateTranslateOutput(dist), 0);
    }

    @Override
    public boolean isFinished() {
        return drive.atTranslationSetpoint();
    }

</details>
<br>


Implement the same logic for the rotation command, but set the applied rotation speed to be the output of the rotation PID controller. 
<details>
    <summary>
        <a href="Solution/src/main/java/frc/robot/commands/TurnCommand.java#L27">Solution</a>
    </summary>

    @Override
    public void execute() {
        drive.arcadeDrive(0, drive.calculateRotOutput(angle));
    }

    @Override
    public boolean isFinished() {
        return drive.atRotationSetpoint();
    }

</details>
<br>

Deploy this new code to your Romi and run these commands from the Sendable Chooser on the Glass dashboard. 