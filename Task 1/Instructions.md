# Task 1: Controlling Romi Drivetrain and Sensors

## Resources
- [WPILib Command based programming structure](https://docs.wpilib.org/en/stable/docs/software/commandbased/structuring-command-based-project.html)
- [The anatomy of a command](https://github.com/wpilibsuite/allwpilib/blob/main/wpilibNewCommands/src/main/java/edu/wpi/first/wpilibj2/command/Command.java)
- [WPILib Romi programming docs](https://docs.wpilib.org/en/stable/docs/romi-robot/programming-romi.html)

## Running code on a romi

When writing code it is easy to make mistakes so being able to test code is very important. After each section of the task you should try running it on a romi. To run code on a romi you need to connect to its wifi network which should have the password `82488248`, after you connect to the romi click on the <img src="./wpilib%20logo.png" alt="wpilib logo" width="35"/>(WPILib VScode extention) in the top right of the screen and chose simulate robot code. After the program starts up you will see the sim window.

<img src="./sim.png" alt="sim window" width="750"/>

The most important sections of the window are the robot mode in the top left, and the joystick sections. The robot mode lets you change the mode of the robot to be differnt things like disconnected, disabled, telop, and autonomous, when you are manualy driving the robot it should be in telop. If you are using a joystick you can bind wasd to a joystick by draging keyboard 0 onto one of the joysticks.
## Conventions
### Naming Conventions
On ChainLynx, we use the following naming conventions

```java
// For classes
public class RobotContainer {}

// For objects,
private Subsystem elevatorSubsystem;

// For constants,

public static final double kMaxVelocity;

// For class fields,

private double speedMultiplier;
```

In other teams or example code, you may seem conventions like m_ObjectName for objects, and other slight variations.
Always remember to read the type and examine the usage to make sure you know what your looking at.

### Units
[The Units library](https://docs.wpilib.org/en/stable/docs/software/basic-programming/java-units.html) alows you have variables like `Distance kElevatorHeight` instead of `double kElevatorHeightMeters` The advantage of this you you can get the height in meters but also in inches or feet, the other main reason to use the units library is to avoid mismatched units like saying that a measurment in feet is in meters like what happened with the [Mars Climate Orbiter](https://en.wikipedia.org/wiki/Mars_Climate_Orbiter#Cause_of_failure).

You can create a measure(such as Distance or Angle) using the `.of` meathod on a unit eg. `Distance kBumperWidth = Inches.of(23.5)` You can also manipulate a measure with meathods like `.plus` or `.times`, and you can compare them with meathods like `.lt`(less than) or `.gte`(greater than or equal to).

To use the units library you can import `import static edu.wpi.first.units.Units.*;` for the units like `Inches` or `Rotations`, and `import edu.wpi.first.units.measure.*;` for measures like `Distance` or `Angle`. When writing code try to use the units library whenever aplicable.


## Objectives 
Off the template command-based Romi project, create a command that will allow joystick or keyboard input to control the Romi drivetrain. Create commands to drive the Romi forward and rotate it a certain amount.

## Task Details - Part 1
### Creating the project
Use the WPILib VSCode extension to create a new **Romi template (not example like the link says)** [command-based project](https://docs.wpilib.org/en/stable/docs/romi-robot/programming-romi.html). You will need to select several options the most important is to enable desktop suport.
### Setting up the constants file
In the constants file you need several things, because the constants are for the drivetrain you should add another class inside of constants called `DriveConstants`. In the DriveConstants class, you will need a `kDefaultDriveSpeed` `kDefaultRotSpeed` `kCountsPerRevolution` and `kWheelDiameter`. The two default speeds should be doubles set to whatever value you want between 0 and 1, and will scale how fast the romi drives and turns, 1 being the fastest and 0 being the slowest. The kCountsPerRevolution helps the encoder track how far the romi has moved, which we will be important later. Finally we need to use the Units library to construct a `Distance` for the Wheel Diameter, and we should say `Millimeters.of(70)` to specify that our wheels are 70 mm in diameter.

<details>
    <summary>
        <a href="Solution/src/main/java/frc/robot/Constants.java#L19">Solution</a>
    </summary>

    public final class DriveConstants {
        public static final double kDefaultDriveSpeed = 0.5; // fraction of max power, 0-1
        public static final double kDefaultRotSpeed = 0.5; // fraction of max power, 0-1
        public static final double kCountsPerRevolution = 1440.0;
        public static final Distance kWheelDiameter = Millimeters.of(70);
    }

</details>
<br>

### Drivetrain
Look at RomiDrivetrain.java in the subsystems folder. For tank drivetrains like Romi, which can’t turn and move back and forth at the same time, we use arcade drive (try to find this method) to control the Romis.

<details>
    <summary>
        <a href="Solution/src/main/java/frc/robot/subsystems/RomiDrivetrain.java#L47">Solution</a>
    </summary>

    public void arcadeDrive(double xaxisSpeed, double zaxisRotate) {
        diffDrive.arcadeDrive(xaxisSpeed, zaxisRotate);
    }

</details>
<br>

Arcade drive is a method of diffDrive, which is inside an instance of the RomiDrivetrain class, when you call Arcade drive on a RomiDrivetrain it will pass the call to the diffDrive. The parameters of arcade drive are xaxisSpeed, the speed of translation, and zaxisRotate, the speed at which we want the robot to rotate. In a differential drivetrain, the difference in the magnitudes of the individual wheel's speeds determines the speed at which the robot will rotate, and the ratios of the speed of each wheel can be used to determine the ratio of translational velocity vs rotational velocity.

If you right click on arcadeDrive called on the RomiDrivetrain instance and click 'go to definition' (F12 Shortcut), you can view the internals of WPILib's differential drive implementation for Romis. Under the hood, the speeds set on each wheel are:
```java
double leftSpeed = xSpeed - zRotation;
double rightSpeed = xSpeed + zRotation;
```
When reading robot code, understanding what each of the variables you are looking is, what their value should be, and where that value is coming from. Look at your code so far and try to figure out what each of these values, taking into account that romis have a diffdrive
T, when one wheel is spinning faster than the other, the robot will rotate in the direction of the wheel that's spinning slower.

### Commands
In WPILib there are two ways of declaring a command the first is more verbose but gives you a better idea of how a command functions under the hood.

To create the first type of commmand, in the commands folder, create a class called DriveCommand that extends the generic Command object, you can then copy the example command for the functions that you will need. We want this command to use joystick input to move the Romi around. The methods in the body of this command are from its parent class, Command, so we use the @Override annotation to signify that we're inheriting logic from the parent class.

<details>
    <summary>
        <a href="Solution/src/main/java/frc/robot/commands/ExampleCommand.java#L5">Solution</a>
    </summary>

    package frc.robot.commands;

    import edu.wpi.first.wpilibj2.command.Command;

    public class DriveCommand extends Command {

        public DriveCommand() {}

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

</details>
<br>

In its execute method (this is periodic, so it is called for every cycle of the Command Scheduler, which is a class that manages the state of the robot program, or every 0.02 seconds) we will be calling the arcadeDrive method from the RomiDrivetrain subsystem based on joystick inputs. 

Normally, we need to track and report if the command is finished with isFinished(), but because when the robot is not doing anything else we want to be able to drive it with joysticks, we can simply let the command scheduler cancel the command when another one needs the drivetrain.


The DriveCommand’s constructor should have the following arguments:
- `RomiDrivetrain drive` (an instance of the drivetrain object, which will be defined in RobotContainer)
- `DoubleSupplier speed` (the speed at which the robot moves forward/backward)
- `DoubleSupplier rotation` (the speed at which the robot turns)

#### Why are we using Double Suppliers instead of just doubles? 
A DoubleSupplier is a [functional interface](https://www.baeldung.com/java-8-functional-interfaces) that generates doubles dynamically (when requested). Primitives(double, int, boolean, ...) won’t change once they’re passed in, but suppliers contain code that gives new doubles on the fly. Therefore, when the joystick input changes, we don’t have to create a whole new command object just to change the command inputs!

<details>
    <summary>
        <a href="Solution/src/main/java/frc/robot/commands/DriveCommand.java#L7">Solution</a>
    </summary>

    private final RomiDrivetrain drive;
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

</details>
<br>

In execute, call the arcadeDrive method with speed and rot as inputs

<details>
    <summary>
        <a href="Solution/src/main/java/frc/robot/commands/DriveCommand.java#L27">Solution</a>
    </summary>

    @Override
    public void execute() {
        drive.arcadeDrive(speed.getAsDouble(), rot.getAsDouble());
    }

</details>
<br>

Moving into the constructor of RobotContainer, initialize RomiDrivetrain as a variable and create a new DriveCommand.Now, initialize a Joystick() in RobotContainer, you can then use the joystick to get double suppliers for the drive command. To make a suplier you need an [anonymous function / lambda](https://www.w3schools.com/java/java_lambda.asp) which is writen with the syntax `() -> someCode` the parenthesies represent the input to the function, so in our case they are empty. After the arrow is the code that will will return a value, in this case a double.

<details>
    <summary>
        <a href="Solution/src/main/java/frc/robot/RobotContainer.java#L29">Solution</a>
    </summary>

    private final RomiDrivetrain romiDrivetrain;
    private final DriveCommand driveCommand;
    private final Joystick joystick = new Joystick(0);

    /** The container for the robot. Contains subsystems, OI devices, and commands. */
    public RobotContainer() {
        romiDrivetrain = new RomiDrivetrain();
        driveCommand = new DriveCommand(romiDrivetrain, () -> joystick.getY(), () -> joystick.getX());
        // Configure the button bindings
        configureButtonBindings();
    }

</details>
<br>
It is often good style to finish object initalization in the constructor of the class you're coding, as your more explicitly specifying that you're initializing those objects when the class itself is being initialized. 

This code instantiates a RomiDrivetrain object, a Joystick object on port 0 of Driver Station, and the Drive Command you just started to create.

Then, in the constructor of RobotContainer, set this command to be the default command for the subsystem, so it will always be running unless it is interrupted by another command. This is why we don’t need an isFinished condition for DriveCommand because it will only ever be interrupted, not terminated. 

<details>
    <summary>
        <a href="Solution/src/main/java/frc/robot/RobotContainer.java#L61">Solution</a>
    </summary>

    romiDrivetrain.setDefaultCommand(driveCommand);

</details>
<br>

When the command runs, it should now automatically reference the controller values and respond to joystick input. Congratulations, this is now a functioning robot!

Now you should connect to the romi over Wifi and try driving it around if your confused on how to connect to the keyboard there is some stuff explaining it at the very top, and remember that once you assign the keyboard to the joystick wasd will control the robot.

## Task Details - Part 2
Now we will use the same command logic to create rotate and translate commands. We will call the arcadeDrive method periodically, and once the translation distance or rotation angle, respectively, is reached (use sensor readings from the drivetrain), return true for the isFinished method.

### Translation command
We will now drive a certain distance using data from the drivetrain's encoders. There are methods in the drivetrain class to get the distances the encoders have traveled (`.getLeftDistance` and `.getRightDistance`). You should make a new file and command class called `TranslateCommand` and can copy the structure from DriveCommand. The code will be similar to the default command we wrote earlier, except we will have an finish condition that terminates the command once the desired distance is reached. We will also drive at a constant speed with no rotation.

#### Subclass command
In the constructor you will need to take in the RomiDrivetrain and the distance to travel. After that you use [Math.signum](https://docs.oracle.com/javase/8/docs/api/java/lang/Math.html#signum-double-), which gets if a number is posative or negative, to determine if we need to go forward or backward to reach our goal distance. Since dist is a `Distance` we need to convert it to a double to be able to use `Math.signum`. To do this we will use `dist.magnitude()` which returns the number that the distance is in, so if the `Distance` is 12 inches, it will return 12.

<details>
    <summary>
        <a href="Solution/src/main/java/frc/robot/commands/TranslateCommand.java#L11">Solution</a>
    </summary>

    private RomiDrivetrain drive;
    private Distance dist;
    // should be 1 or -1
    private double dir;

    public TranslateCommand(RomiDrivetrain drive, Distance dist) {
        this.drive = drive;
        this.dist = dist;

        this.dir = Math.signum(dist.magnitude());

        addRequirements(drive);
    }

</details>
<br>


After defining the constructor to access the drivetrain instance defined in Robot Container and the desired distance to travel (this is called dependency injection), we need to reset the encoder measurements to be zeroed relative to the start of the command and drive the robot at a constant translational speed until it reaches its distance setpoint. To do this we use the `resetEncoders()` method from the drivetrain, and since we have an instance of the drivetrain in the command, we can access it using `drive.resetEncoders();` and put this in the `initialize()` part of the command.

<details>
    <summary>
        <a href="Solution/src/main/java/frc/robot/commands/TranslateCommand.java#L25">Solution</a>
    </summary>

    @Override
    public void initialize() {
        drive.resetEncoders();
    }

</details>
<br>

Next we neeed to fill in the `execute()` `isFinished()` and `end()` methods. 
We need a simmilar idea in `execute()` as their was in `DriveCommand` but now we are moving at a constant speed not turning at all. Our speed will just be `kDefaultDriveSpeed * dir` and our turn will be `0`.

For `isFinished()` its more complicated. We want our romi to stop moving, aka finishing the command, when we have reached our goal distance. 

The first thing we need to do is create a method to get an average distance of our two encoders. We need to make this method in the same class with our encoders, `RomiDrivetrain`. First, convert `getLeftEncoderDistance()` and `getRightEncoderDistance()` to returning a `Distance` instead of a double.

To do this, we first change the type `double` to `Distance`, add `Inches.of()` to the front of the return of each method, to have it return as inches instead of as a double.

Then for the new `getAverageDistance()` method we need for the command, we need to add the `getLeftDistanceInch()` and the `getRightDistanceInch()` using `.plus()` and then divide by 2, using `.div(2)` remember to add parenthesies because java sometimes cares about PEMDAS.
<details>
    <summary>
        <a href="Solution/src/main/java/frc/robot/subsystems/RomiDrivetrain.java#L30">Solution</a>
    </summary>

    public Distance getLeftDistanceInch() {
        return Inches.of(m_leftEncoder.getDistance());
    }

    public Distance getRightDistanceInch() {
        return Inches.of(m_rightEncoder.getDistance());
    }

    public Distance getAverageDistance() {
        return (getLeftDistanceInch().plus(getRIghtDistanceInch())).div(2);
    }

</details>
<br>

To do this we will compare the average of our two encoders `drive.getAverageDistance()` with `dist` using the unit based operator `.gte()`.

Finally for `end()` which runs when the command ends, we want to stop the drivetrain, which we will do using `drive.arcadeDrive(0,0)`.

<details>
    <summary>
        <a href="Solution/src/main/java/frc/robot/subsystems/RomiDrivetrain.java#L30">Solution</a>
    </summary>

    @Override
    public void execute() {
        drive.arcadeDrive(kDefaultDriveSpeed * dir, 0);
    }

    @Override
    public boolean isFinished() {
        return drive.getAverageDistance().gte(dist);
    }

    @Override
    public void end(boolean interrupted) {
        drive.arcadeDrive(0, 0);
    }

</details>
<br>

### Rotation command
This command is logically very similar to the translation command. We need to define a RomiGyro object in the drivetrain subsystem, as well as a function to get its angle, which is in degrees by default so you will need to convert it to an angle object. This function will use `gyro.getAngle()` and just like the encoder values we will need to return an angle, using `Degrees.of()`. We also need to be able to reset the gyro, so make another method called `resetGyro()` that runs `gyro.reset()`.

<details>
    <summary>
        <a href="Solution/src/main/java/frc/robot/subsystems/RomiDrivetrain.java#L29">Solution</a>
    </summary>

    private final RomiGyro gyro = new RomiGyro();

    public Angle getAngle() {
        return Degrees.of(gyro.getAngle());
    }

    public void resetGyro() {
        gyro.reset();
    }

</details>
<br>

#### Command class
Now create a file and command class called TurnCommand and copy the contents of `TranslateCommand`.
The first thing to do is change every reference of dist to angle, because these commands are trying to do the same thing but instead of a goal distance traveled, we have a goal angle turned. We need an `Angle angle` in the constructor, and we should define `dir` in the same way using `Math.signum()` again. In `initialize()` we now reset the gyro instead of the encoders, and in `execute()` we use `kDefaultRotSpeed * dir` instead of `kDefaultDriveSpeed`. Finally, in `isFinished()` we instead compare `drive.getAngle()` with the `angle` instead of the encoder distance with the distance. `end()` will stay exactly the same.

<details>
    <summary>
        <a href="Solution/src/main/java/frc/robot/commands/TurnCommand.java#L24">Solution</a>
    </summary>

    public TurnCommand(RomiDrivetrain drive, Angle angle) {
        this.drive = drive;
        this.angle = angle;
        dir = Math.signum(angle.magnitude());
        addRequirements(drive);
    }

    @Override
    public void initialize() {
        drive.resetGyro();
    }

    @Override
    public void execute() {
        drive.arcadeDrive(0, kDefaultRotSpeed * dir);
    }

    @Override
    public boolean isFinished() {
        return drive.getAngle().gte(angle);
    }

    @Override
    public void end(boolean interrupted) {
        drive.arcadeDrive(0, 0);
    }

</details>
<br>

Now you also have a rotation command! Now just like `DriveCommand` we need to make an instance of `TranslateCommand` and `TurnCommand` in `RobotContainer`. For constructing the commands, you may want to make a `kDefaultDistance` and a `kDefaultAngle` in DriveConstants, and pass them into the command constructors.

### Running your commands
In Robot Container, create a [Sendable Chooser](https://docs.wpilib.org/en/stable/docs/software/dashboards/smartdashboard/choosing-an-autonomous-program-from-smartdashboard.html) that consumes commands (SendableChooser<Command>) to add different autonomous options. 

<details>
    <summary>
        <a href="Solution/src/main/java/frc/robot/RobotContainer.java#L42">Solution</a>
    </summary>

    private final SendableChooser<Command> autoChooser = new SendableChooser<>();

</details>

In the constructor of `RobotContainer` use `chooser.addOption()` to add your translate and turn commands to the `SendableChooser`. Finally call `SmartDashboard.putData(chooser)` to send the commands to the dash board.
<details>
    <summary>
        <a href="Solution/src/main/java/frc/robot/RobotContainer.java#L64">Solution</a>
    </summary>

    chooser.addOption("drive 6 inches", translateCommand);
    chooser.addOption("turn 180", turnCommand);
    SmartDashboard.putData(chooser);
    
</details>

Then, at the bottom of `RobotContainer` have the method `getAutonomousCommand()` return `chooser.getSelected()`.

To get them to run, start the romi, and look through the tabs at the top until you see SmartDashboard, and select the sendable chooser. Select the command that you want to run, and then set the romi into `Autonomous` in the top right, and if everything went well your command should run!
