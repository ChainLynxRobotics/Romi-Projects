package frc.robot.opmodes;

import frc.robot.Robot;
import org.wpilib.command3.Command;
import org.wpilib.command3.Scheduler;
import org.wpilib.opmode.Autonomous;
import org.wpilib.opmode.OpMode;

/** Shows up as "Square PID" in the Autonomous list of the sim window. */
@Autonomous(name = "Square PID")
public class SquarePIDAuto implements OpMode {
  private final Robot robot;

  public SquarePIDAuto(Robot robot) {
    this.robot = robot;
  }

  /** Runs once when the robot is enabled. */
  @Override
  public void start() {
    var drivetrain = robot.drivetrain;

    Command square =
        Command.noRequirements(coroutine -> {
              for (int side = 0; side < 4; side++) {
                coroutine.await(drivetrain.driveForwardPID(12)); // run it and wait until it's done
                coroutine.await(drivetrain.turnLeftPID(90));
              }
            })
            .named("Square PID");

    Scheduler.getDefault().schedule(square);
  }
}
