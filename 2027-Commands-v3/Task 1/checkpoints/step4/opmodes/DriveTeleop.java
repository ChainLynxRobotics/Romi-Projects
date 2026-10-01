package frc.robot.opmodes;

import frc.robot.Robot;
import org.wpilib.opmode.OpMode;
import org.wpilib.opmode.Teleop;

/** Shows up as "Drive" in the Teleop list of the sim window. */
@Teleop(name = "Drive")
public class DriveTeleop implements OpMode {
  public DriveTeleop(Robot robot) {
    var drivetrain = robot.drivetrain;
    var controller = robot.controller;

    // Joysticks are backwards (pushing up gives a negative number), so flip them with "-"
    drivetrain.setDefaultCommand(
        drivetrain.driveWithJoysticks(
            () -> -controller.getLeftY(), () -> -controller.getRightX()));

    // A button (Z on the keyboard): drive forward 12 inches
    controller.faceDown().onTrue(drivetrain.driveForward(12));

    // B button (X on the keyboard): turn left 90 degrees
    controller.faceRight().onTrue(drivetrain.turnLeft(90));
  }
}
