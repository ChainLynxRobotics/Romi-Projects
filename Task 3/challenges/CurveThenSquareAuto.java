package frc.robot.opmodes;

import frc.robot.Robot;
import java.util.List;
import org.wpilib.command3.Command;
import org.wpilib.command3.Scheduler;
import org.wpilib.math.geometry.Pose2d;
import org.wpilib.math.geometry.Rotation2d;
import org.wpilib.math.geometry.Translation2d;
import org.wpilib.math.trajectory.DrivetrainSplineTrajectory;
import org.wpilib.opmode.Autonomous;
import org.wpilib.opmode.OpMode;

/** Shows up as "Curve then Square": the S-curve, then Square PID. */
@Autonomous(name = "Curve then Square")
public class CurveThenSquareAuto implements OpMode {
  private final Robot robot;
  private final DrivetrainSplineTrajectory path;

  public CurveThenSquareAuto(Robot robot) {
    this.robot = robot;
    path =
        robot.drivetrain.makePath(
            new Pose2d(0.5, 0.5, Rotation2d.ZERO),
            List.of(new Translation2d(0.9, 0.8), new Translation2d(1.3, 0.3)),
            new Pose2d(1.8, 0.6, Rotation2d.ZERO));
    robot.drivetrain.showPath(path);
  }

  @Override
  public void start() {
    var drivetrain = robot.drivetrain;
    drivetrain.resetPose(path.start().pose);

    Command auto =
        Command.noRequirements(coroutine -> {
              coroutine.await(drivetrain.followPath(path));
              for (int side = 0; side < 4; side++) {
                coroutine.await(drivetrain.driveForwardPID(12));
                coroutine.await(drivetrain.turnLeftPID(90));
              }
            })
            .named("Curve then Square");

    Scheduler.getDefault().schedule(auto);
  }
}
