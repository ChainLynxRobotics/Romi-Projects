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

/** Shows up as "Figure 8": a loop on the right, then a loop on the left. */
@Autonomous(name = "Figure 8")
public class FigureEightAuto implements OpMode {
  private final Robot robot;
  private final DrivetrainSplineTrajectory rightLoop;
  private final DrivetrainSplineTrajectory leftLoop;

  public FigureEightAuto(Robot robot) {
    this.robot = robot;
    var drivetrain = robot.drivetrain;

    rightLoop =
        drivetrain.makePath(
            new Pose2d(0.5, 0.5, Rotation2d.ZERO),
            List.of(
                new Translation2d(1.25, 0.75),
                new Translation2d(1.7, 1.05),
                new Translation2d(2.0, 0.75),
                new Translation2d(1.7, 0.45)),
            new Pose2d(1.25, 0.75, Rotation2d.fromDegrees(135)));

    leftLoop =
        drivetrain.makePath(
            new Pose2d(1.25, 0.75, Rotation2d.fromDegrees(135)),
            List.of(new Translation2d(0.8, 1.05), new Translation2d(0.5, 0.75), new Translation2d(0.8, 0.45)),
            new Pose2d(1.25, 0.75, Rotation2d.fromDegrees(45)));

    drivetrain.showPath(rightLoop);
  }

  @Override
  public void start() {
    var drivetrain = robot.drivetrain;
    drivetrain.resetPose(rightLoop.start().pose);

    Command auto =
        Command.noRequirements(coroutine -> {
              coroutine.await(drivetrain.followPath(rightLoop));
              coroutine.await(drivetrain.followPath(leftLoop));
            })
            .named("Figure 8");

    Scheduler.getDefault().schedule(auto);
  }
}
