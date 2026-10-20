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

/** Shows up as "There and Back": drive a curve out, spin around, drive a curve back. */
@Autonomous(name = "There and Back")
public class ThereAndBackAuto implements OpMode {
  private final Robot robot;
  private final DrivetrainSplineTrajectory there;
  private final DrivetrainSplineTrajectory back;

  public ThereAndBackAuto(Robot robot) {
    this.robot = robot;
    var drivetrain = robot.drivetrain;

    there =
        drivetrain.makePath(
            new Pose2d(0.5, 0.5, Rotation2d.ZERO),
            List.of(new Translation2d(1.0, 0.9)),
            new Pose2d(1.5, 0.5, Rotation2d.ZERO));

    // Starts facing backwards (180 degrees) where "there" ended
    back =
        drivetrain.makePath(
            new Pose2d(1.5, 0.5, Rotation2d.fromDegrees(180)),
            List.of(new Translation2d(1.0, 0.2)),
            new Pose2d(0.5, 0.5, Rotation2d.fromDegrees(180)));

    drivetrain.showPath(there);
  }

  @Override
  public void start() {
    var drivetrain = robot.drivetrain;
    drivetrain.resetPose(there.start().pose);

    Command auto =
        Command.noRequirements(coroutine -> {
              coroutine.await(drivetrain.followPath(there));
              coroutine.await(drivetrain.turnLeftPID(180));
              coroutine.await(drivetrain.followPath(back));
            })
            .named("There and Back");

    Scheduler.getDefault().schedule(auto);
  }
}
