package frc.robot.opmodes;

import frc.robot.Robot;
import org.wpilib.command3.Command;
import org.wpilib.command3.Scheduler;
import org.wpilib.opmode.Autonomous;
import org.wpilib.opmode.OpMode;

/** Shows up as "Square" in the Autonomous list of the sim window. */
@Autonomous(name = "Triangle")
public class TriangleAuto implements OpMode {
  private final Robot robot;

  public TriangleAuto(Robot robot) {
    this.robot = robot;
  }

  /** Runs once when the robot is enabled. */
  @Override
  public void start() {
    var drivetrain = robot.drivetrain;

    Command square =
        Command.noRequirements(coroutine -> {
              for (int side = 0; side < 3; side++) {
                coroutine.await(drivetrain.driveForward(12)); // run it and wait until it's done
                coroutine.await(drivetrain.turnLeft(120));
              }
            })
            .named("Triangle");

    Scheduler.getDefault().schedule(square);
  }
}
