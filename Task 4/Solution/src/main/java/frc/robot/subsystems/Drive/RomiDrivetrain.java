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

  private final DifferentialDrive m_diffDrive;
  private final Wheel m_leftWheel;
  private final Wheel m_rightWheel;

  private final Gyro gyro;
  private final DifferentialDriveOdometry m_odometry;

  private final PIDController m_rotController;
  private final PIDController m_translateController;

  public RomiDrivetrain(WheelIO left, WheelIO right, GyroIO gryo) {
    m_leftWheel = new Wheel(left, 0);
    m_rightWheel = new Wheel(right, 1);
    m_diffDrive = new DifferentialDrive(m_leftWheel::set, m_rightWheel::set);

    this.gyro = new Gyro(gryo);

    m_odometry =
        new DifferentialDriveOdometry(
            new Rotation2d(gyro.getAngle()),
            m_leftWheel.getPosition(),
            m_rightWheel.getPosition());

    switch (Constants.currentMode) {
      case REAL:
      case REPLAY:
        m_translateController = new PIDController(0.05, 0.0, 0.0);
        m_rotController = new PIDController(7.0, 0.0, 0.0);
        break;
      case SIM:
        m_translateController = new PIDController(0.1, 0.0, 0.0);
        m_rotController = new PIDController(10.0, 0.0, 0.0);
        break;
      default:
        m_translateController = new PIDController(0.1, 0.0, 0.0);
        m_rotController = new PIDController(10.0, 0.0, 0.0);
        break;
    }
  }

  public void arcadeDrive(double xaxisSpeed, double zaxisRotate) {
    m_diffDrive.arcadeDrive(xaxisSpeed, zaxisRotate);
  }

  public void driveChassisSpeeds(ChassisSpeeds speeds) {
    arcadeDrive(
        speeds.vxMetersPerSecond / kMaxSpeed.in(MetersPerSecond),
        speeds.omegaRadiansPerSecond / kMaxSpeed.in(MetersPerSecond)
    );
  }

  public void resetEncoders() {
    m_leftWheel.resetEncoder();
    m_rightWheel.resetEncoder();
  }

  public Distance getLeftDistance() {
    return m_leftWheel.getPosition();
  }

  public Distance getRightDistance() {
    return m_rightWheel.getPosition();
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
    return m_odometry.getPoseMeters();
  }

  public double calculateRotOutput(double curRot, double setpoint) {
    return m_rotController.calculate(curRot, setpoint);
  }

  public double calculateTranslateOutput(double curDist, double setpoint) {
    return m_translateController.calculate(curDist, setpoint);
  }


  public boolean atTranslationSetpoint() {
    return m_translateController.atSetpoint();
  }

  public boolean atRotationSetpoint() {
    return m_rotController.atSetpoint();
  }

  public ChassisSpeeds getSpeeds() {
    return diffDriveKinimatics.toChassisSpeeds(
        new DifferentialDriveWheelSpeeds(m_leftWheel.getVelocity(), m_rightWheel.getVelocity()));
  }

  public void resetOdometry(Pose2d pose) {
    m_odometry.resetPosition(
        new Rotation2d(gyro.getAngle()),
        new DifferentialDriveWheelPositions(m_leftWheel.getPosition(), m_rightWheel.getPosition()),
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
    m_leftWheel.periodic();
    m_rightWheel.periodic();
    m_leftWheel.updateInputs();
    m_rightWheel.updateInputs();
    gyro.updateInputs();
    gyro.periodic();

    Logger.recordOutput("gyro angle", gyro.getAngle());
    Logger.recordOutput("left wheel", m_leftWheel.getPosition());
    Logger.recordOutput("right wheel", m_leftWheel.getPosition());
    m_odometry.update(
        new Rotation2d(gyro.getAngle()), m_leftWheel.getPosition().in(Meters), m_rightWheel.getPosition().in(Meters));
  }
}
