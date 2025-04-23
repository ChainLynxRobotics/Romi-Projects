# Task 1: Controlling Romi Drivetrain and Sensors

### Objectives 
Off the template command-based Romi project, create a command that will allow joystick or keyboard input to control the Romi drivetrain. Create commands to drive the Romi forward and rotate it a certain amount.

### Resources
- [WPILib Command based programming structure](https://docs.wpilib.org/en/stable/docs/software/commandbased/structuring-command-based-project.html)
- [The anatomy of a command](https://github.com/wpilibsuite/allwpilib/blob/main/wpilibNewCommands/src/main/java/edu/wpi/first/wpilibj2/command/Command.java)
- [WPILib Romi programming docs](https://docs.wpilib.org/en/stable/docs/romi-robot/programming-romi.html)
- [WPILib PID controllers](https://docs.wpilib.org/en/stable/docs/software/advanced-controls/controllers/pidcontroller.html)


### Task Details - Part 1
#### Naming Conventions
At ChainLynx, we use the following naming conventions

```
// For Objects,
private Subsystem subsystem;

// For Constants,

public static final kMaxVelocity;

// For Fields,

private double multiplier;
```

In other teams or example code, you may seem conventions like m_ObjectName for objects, and other slight variations.
Always remember to read the type and examine the usage to make sure you know what your looking at.
#### Drivetrain
Once you have used the WPILib VSCode extension to create a new [Romi template (not example) command-based project](https://docs.wpilib.org/en/stable/docs/romi-robot/programming-romi.html), look at RomiDrivetrain.java in the subsystems folder. For tank drivetrains like Romi, which can’t turn and move back and forth at the same time, we use arcade drive (try to find this method) to control the Romis. 

```
public void arcadeDrive(double xaxisSpeed, double zaxisRotate) {
    diffDrive.arcadeDrive(xaxisSpeed, zaxisRotate);
}
```

Arcade drive is a method of diffDrive, which is inside an instance of the RomiDrivetrain class, when you call Arcade drive on a RomiDrivetrain it will pass the call to the diffDrive. The parameters of arcade drive are xaxisSpeed, the speed of translation, and zaxisRotate, the speed at which we want the robot to rotate. In a differential drivetrain, the difference in the magnitudes of the individual wheel's speeds determines the speed at which the robot will rotate, and the ratios of the speed of each wheel can be used to determine the ratio of translational velocity vs rotational velocity.

If you right click on arcadeDrive called on the RomiDrivetrain instance and click 'go to definition' (F12 Shortcut), you can view the internals of WPILib's differential drive implementation for Romis. Under the hood, the speeds set on each wheel are:
```
double leftSpeed = xSpeed - zRotation;
double rightSpeed = xSpeed + zRotation;
```
When reading robot code, understanding what each of the variables you are looking is, what their value should be, and where that value is coming from. Look at your code so far and try to figure out what each of these values, taking into account that romis have a diffdrive
T, when one wheel is spinning faster than the other, the robot will rotate in the direction of the wheel that's spinning slower.

Now, in the commands folder, create a class called DriveCommand that extends the generic Command object. We want this command to use joystick input to move the Romi around. The methods in the body of this command are from its parent class, Command, so we use the @Override annotation to signify that we're inheriting logic from the parent class.

```
public class DriveCommand extends Command {
    // Called when the command is initially scheduled.
    @Override
    public void initialize() {}

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {}

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {}

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;
    }
}
```

In its execute method (this is periodic, so it is called for every cycle of the Command Scheduler, which is a class that manages the state of the robot program, or every 0.02 seconds), we will be calling the arcadeDrive method from the RomiDrivetrain subsystem based on joystick inputs. 

Normally, we need to track and report if the command is finished with isFinished(), but this command is an exception which will be explained shortly.


The DriveCommand’s constructor should have the following arguments:
- *RomiDrivetrain* drivetrain (an instance of the drivetrain object, which will be defined in RobotContainer)
- *DoubleSupplier* speed (the speed at which the robot moves forward/backward)
- *DoubleSupplier* rotation (the speed at which the robot turns)

#### Why are we using Double Suppliers instead of just doubles? 
A DoubleSupplier is a functional interface that generates doubles dynamically (when requested). Primitives like doubles won’t change once they’re passed in, but suppliers contain code that gives new doubles on the fly. Therefore, when the joystick input changes, we don’t have to create a whole new command object just to change the command inputs!

```
private final RomiDrivetrain drivetrain;
private final DoubleSupplier speed;
private final DoubleSupplier rot;

public DriveCommand(
    RomiDrivetrain drive, DoubleSupplier speed, DoubleSupplier rot) {
    this.drive = drive;
    this.speed = speed;
    this.rot = rot;

    // Use addRequirements here to declare drive dependencies.
    addRequirements(drive);
}
```

In execute, call the arcadeDrive method with speed and rot as inputs
```
@Override
public void execute() {
    drive.arcadeDrive(speed.getAsDouble(), rot.getAsDouble());
}
```

Moving into the constructor of RobotContainer, initialize RomiDrivetrain as a variable and create a new DriveCommand. Now, initialize a Joystick() in RobotContainer, and pass in the controller X and Y values into DriveCommand through the earlier suppliers. 

```
private final RomiDrivetrain romiDrivetrain;
private final DriveCommand autoCommand;
private final Joystick stick = new Joystick(0);

/** The container for the robot. Contains subsystems, OI devices, and commands. */
public RobotContainer() {
    romiDrivetrain = new RomiDrivetrain();
    autoCommand = new DriveCommand(romiDrivetrain, () -> stick.getRawAxis(0), () -> stick.getRawAxis(1));
    // Configure the button bindings
    configureButtonBindings();
}
```
It is often good style to finish object initalization in the constructor of the class you're coding, as your more explicitly specifying that you're initializing those objects when the class itself is being initialized. 

This code instantiates a RomiDrivetrain object, a Joystick object on port 0 of Driver Station, and the Drive Command you just started to create. Note that by using () ->, which is an [anonymous function](https://www.w3schools.com/java/java_lambda.asp), you are turning a regular double into a Double Supplier. By calling getRawAxis, you can access the x, y, z, and w axes of a given joystick.

Then, in the constructor of RobotContainer, set this command to be the default command for the subsystem, so it will always be running unless it is interrupted by another command. This is why we don’t need an isFinished condition for DriveCommand because it will only ever be interrupted, not terminated. 
```
romiDrivetrain.setDefaultCommand(command);
```

When the command runs, it should now automatically reference the controller values and respond to joystick input. Congratulations, this is now a functioning robot!

### Task Details - Part 2
Now we will use the same command logic to create rotate and translate commands. We will call the arcadeDrive method periodically, and once the translation distance or rotation angle, respectively, is reached (use sensor readings from the drivetrain), return true for the isFinished method.

#### Translation command
We will now drive a certain distance using data from the drivetrain's encoders. There are methods in the drivetrain class to get the distances the encoders have traveled in inches (getLeftDistanceInch and getRightDistanceInch). The code will be similar to the default command we wrote earlier, except we will have an isFinished condition that terminates the command once the desired distance is reached. We will also drive at a constant speed with no rotation.

```
private RomiDrivetrain drive;
    private double dist;

    public TranslateCommand(RomiDrivetrain drive, double distInches) {
        this.drive = drive;
        this.dist = distInches;

        addRequirements(drive);
    }
```

After defining the constructor to access the drivetrain instance defined in Robot Container and the desired distance to travel (this is called dependency injection), we need to reset the encoder measurements to be zeroed relative to the start of the command and drive the robot at a constant translational speed until it reaches its distance setpoint.

```
@Override
public void initialize() {
    drive.resetEncoders();
}

@Override
public void execute() {
    drive.arcadeDrive(Constants.kDefaultDriveSpeed * Math.signum(dist), 0);
}

@Override
public boolean isFinished() {
    return Math.abs((drive.getLeftDistanceInch() + drive.getRightDistanceInch()) / 2) >= Math.abs(dist);
}

@Override
public void end(boolean interrupted) {
    drive.arcadeDrive(0, 0);
}
```
We use absolute values in the isFinished method in the case the inputted distance is negative. Similarly, we multiply the default translation speed by the sign of the distance so we travel in the right direction. Before ending the command, we stop the drivetrain.

#### Rotation command
This command is logically very similar to the translation command. We need to define a RomiGyro object in the drivetrain subsystem, as well as a function to get its angle, which is in degrees by default. 
```
private final RomiGyro gyro = new RomiGyro();

public double getAngle() {
    return gyro.getAngle();
}

public void resetGyro() {
    gyro.reset();
}
```

Now, we can use this method in our turn command.
```
@Override
public void initialize() {
    drive.resetGyro();
}

@Override
public void execute() {
    drive.arcadeDrive(0, Constants.kDefaultRotSpeed * Math.signum(angle));
}

@Override
public boolean isFinished() {
    return Math.abs(drive.getAngle()) >= Math.abs(angle);
}
```
Now you also have a rotation command!

#### Running your commands
In Robot Container, create a [Sendable Chooser](https://docs.wpilib.org/en/stable/docs/software/dashboards/smartdashboard/choosing-an-autonomous-program-from-smartdashboard.html) that consumes commands (SendableChooser<Command>) to add different autonomous options. Add a rotate as well as a translate command as auto options.

```
private final SendableChooser<Command> autoChooser = new SendableChooser<>();
```

In the constructor of Robot Container, add instances of your new commands as options of the auto chooser.
```
autoChooser.addOption("drive 6 inches", new TranslateCommand(romiDrivetrain, 6));
autoChooser.addOption("turn 180", new TurnCommand(romiDrivetrain, 180));
```

Great job finishing your first task!
