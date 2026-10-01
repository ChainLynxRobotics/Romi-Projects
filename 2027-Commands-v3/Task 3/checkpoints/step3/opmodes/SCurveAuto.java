package frc.robot.opmodes;

import frc.robot.Robot;
import java.util.List;
import org.wpilib.math.geometry.Pose2d;
import org.wpilib.math.geometry.Rotation2d;
import org.wpilib.math.geometry.Translation2d;
import org.wpilib.command3.Scheduler;
import org.wpilib.math.trajectory.DrivetrainSplineTrajectory;
import org.wpilib.opmode.Autonomous;
import org.wpilib.opmode.OpMode;

/** Shows up as "S-Curve" in the Autonomous list of the sim window. */
@Autonomous(name = "S-Curve")
public class SCurveAuto implements OpMode {
  private final Robot robot;
  private final DrivetrainSplineTrajectory path;

  /** Runs when you pick "S-Curve" in the list (even while disabled). */
  public SCurveAuto(Robot robot) {
    this.robot = robot;

    path =
        robot.drivetrain.makePath(
            new Pose2d(0.5, 0.5, Rotation2d.ZERO), // start
            List.of(new Translation2d(0.9, 0.8), new Translation2d(1.3, 0.3)), // go through these
            new Pose2d(1.8, 0.6, Rotation2d.ZERO)); // end

    robot.drivetrain.showPath(path);
  }

  /** Runs once when the robot is enabled. */
  @Override
  public void start() {
    robot.drivetrain.resetPose(path.start().pose); // "the Romi is at the start of the path"
    Scheduler.getDefault().schedule(robot.drivetrain.followPath(path));
  }
}
