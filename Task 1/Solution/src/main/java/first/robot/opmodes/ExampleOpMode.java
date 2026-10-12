package first.robot.opmodes;

import first.robot.Robot;

import org.wpilib.driverstation.XboxController;
import org.wpilib.opmode.PeriodicOpMode;
import org.wpilib.opmode.Teleop;

@Teleop
public class ExampleOpMode extends PeriodicOpMode {
  private final Robot robot;
  private final XboxController driveController = new XboxController(0);

  public ExampleOpMode(Robot robot) {
    this.robot = robot;
  }

  @Override
  public void start() {
    // Code to run once when the robot is enabled
  }

  @Override
  public void periodic() {
    // Runs every 20ms when enabled
  }
}
