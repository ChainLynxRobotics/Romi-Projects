package frc.robot.subsystems.Drive;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelPositions;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

import edu.wpi.first.units.measure.*;
import static edu.wpi.first.units.Units.*;

import static frc.robot.Constants.DriveConstants.*;

import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class RomiDrivetrain extends SubsystemBase {

  private final DifferentialDrive diffDrive;
  private final Wheel leftWheel;
  private final Wheel rightWheel;

  private final Gyro gyro;
  private final DifferentialDriveOdometry odometry;

  private final PIDController rotController;
  private final PIDController translateController;

  public RomiDrivetrain(WheelIO left, WheelIO right, GyroIO gryo) {
    leftWheel = new Wheel(left, 0);
    rightWheel = new Wheel(right, 1);
    diffDrive = new DifferentialDrive(leftWheel::set, rightWheel::set);

    this.gyro = new Gyro(gryo);

    odometry =
        new DifferentialDriveOdometry(
            new Rotation2d(gyro.getAngle()),
            leftWheel.getPosition(),
            rightWheel.getPosition());

    switch (Constants.currentMode) {
      case REAL:
      case REPLAY:
        translateController = new PIDController(0.05, 0.0, 0.0);
        rotController = new PIDController(7.0, 0.0, 0.0);
        break;
      case SIM:
        translateController = new PIDController(0.1, 0.0, 0.0);
        rotController = new PIDController(10.0, 0.0, 0.0);
        break;
      default:
        translateController = new PIDController(0.1, 0.0, 0.0);
        rotController = new PIDController(10.0, 0.0, 0.0);
        break;
    }
  }

  public void arcadeDrive(double xaxisSpeed, double zaxisRotate) {
    diffDrive.arcadeDrive(xaxisSpeed, zaxisRotate);
  }

  public void driveChassisSpeeds(ChassisSpeeds speeds) {
    arcadeDrive(
        speeds.vxMetersPerSecond / kMaxSpeed.in(MetersPerSecond),
        speeds.omegaRadiansPerSecond / kMaxSpeed.in(MetersPerSecond)
    );
  }

  public void resetEncoders() {
    leftWheel.resetEncoder();
    rightWheel.resetEncoder();
  }

  public Distance getLeftDistance() {
    return leftWheel.getPosition();
  }

  public Distance getRightDistance() {
    return rightWheel.getPosition();
  }

  public Distance getAverageDistance() {
    return getLeftDistance().plus(getRightDistance()).div(2);
  }

  public void resetGyro() {
    gyro.resetGyro();
  }

  public Angle getAngle() {
    return gyro.getAngle();
  }

  @AutoLogOutput(key = "Odometry/Robot")
  public Pose2d getPose() {
    return odometry.getPoseMeters();
  }

  public double calculateRotOutput(double curRot, double setpoint) {
    return rotController.calculate(curRot, setpoint);
  }

  public double calculateTranslateOutput(double curDist, double setpoint) {
    return translateController.calculate(curDist, setpoint);
  }


  public boolean atTranslationSetpoint() {
    return translateController.atSetpoint();
  }

  public boolean atRotationSetpoint() {
    return rotController.atSetpoint();
  }

  public ChassisSpeeds getSpeeds() {
    return kDiffDriveKinimatics.toChassisSpeeds(
        new DifferentialDriveWheelSpeeds(leftWheel.getVelocity(), rightWheel.getVelocity()));
  }

  public void resetOdometry(Pose2d pose) {
    odometry.resetPosition(
        new Rotation2d(gyro.getAngle()),
        new DifferentialDriveWheelPositions(leftWheel.getPosition(), rightWheel.getPosition()),
        pose);
  }

  public boolean allianceCheck() {
    var alliance = DriverStation.getAlliance();
    if (alliance.isPresent()) {
      return alliance.get() == DriverStation.Alliance.Red;
    }
    return false;
  }

  @Override
  public void periodic() {
    leftWheel.periodic();
    rightWheel.periodic();
    leftWheel.updateInputs();
    rightWheel.updateInputs();
    gyro.updateInputs();
    gyro.periodic();

    Logger.recordOutput("gyro angle", gyro.getAngle());
    Logger.recordOutput("left wheel", leftWheel.getPosition());
    Logger.recordOutput("right wheel", leftWheel.getPosition());
    odometry.update(
        new Rotation2d(gyro.getAngle()), leftWheel.getPosition().in(Meters), rightWheel.getPosition().in(Meters));
  }
}
