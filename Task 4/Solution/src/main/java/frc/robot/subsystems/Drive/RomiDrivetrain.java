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
import edu.wpi.first.wpilibj.BuiltInAccelerometer;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.romi.RomiGyro;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.Mode;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Radians;

import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class RomiDrivetrain extends SubsystemBase {

  private final DifferentialDrive m_diffDrive;
  private final Wheel m_leftWheel;
  private final Wheel m_rightWheel;

  // The Romi has onboard encoders that are hardcoded
  // to use DIO pins 4/5 and 6/7 for the left and right
  private final Encoder leftEncoder = new Encoder(4, 5);
  private final Encoder rightEncoder = new Encoder(6, 7);

  private final RomiGyro m_gyro;
  private final BuiltInAccelerometer m_accelerometer;
  private final DifferentialDriveOdometry m_odometry;

  private final PIDController m_rotController;
  private final PIDController m_translateController;
  private double m_rotation; // degrees

  public RomiDrivetrain(WheelIO left, WheelIO right) {
    m_leftWheel = new Wheel(left, 0);
    m_rightWheel = new Wheel(right, 1);
    m_diffDrive = new DifferentialDrive(m_leftWheel::set, m_rightWheel::set);

    // Use inches as unit for encoder distances
    leftEncoder.setDistancePerPulse(DriveConstants.kWheelDiameter.times(Math.PI).div(DriveConstants.kCountsPerRevolution).in(Meters));
    rightEncoder.setDistancePerPulse(DriveConstants.kWheelDiameter.times(Math.PI).div(DriveConstants.kCountsPerRevolution).in(Meters));

    m_gyro = new RomiGyro();
    m_gyro.reset();
    m_accelerometer = new BuiltInAccelerometer();

    if (Constants.currentMode == Mode.SIM) {
      m_rotation = 0;
    } else {
      m_rotation = m_gyro.getAngle();
    }

    m_odometry =
        new DifferentialDriveOdometry(
            new Rotation2d(m_rotation),
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
        speeds.vxMetersPerSecond / Constants.AutoConstants.kMaxSpeedMetersPerSecond,
        speeds.omegaRadiansPerSecond / Constants.AutoConstants.kMaxAngularSpeedRadiansPerSecond);
  }

  public void resetEncoders() {
    m_leftWheel.resetEncoder();
    m_rightWheel.resetEncoder();
  }

  public double getLeftDistanceMeter() {
    return m_leftWheel.getPosition();
  }

  public double getRightDistanceMeter() {
    return m_rightWheel.getPosition();
  }

  public Distance getLeftDistance() {
    return Meters.of(leftEncoder.getDistance());
  }

  public Distance getRightDistance() {
    return Meters.of(rightEncoder.getDistance());
  }

  public Distance getAverageDistance() {
    return getLeftDistance().plus(getRightDistance()).div(2);
  }

  public void resetGyro() {
    m_gyro.reset();
  }

  public Angle getAngle() {
    return Radians.of(m_rotation);
  }

  public double getAccelerationX() {
    return m_accelerometer.getX();
  }

  public double getAccelerationY() {
    return m_accelerometer.getY();
  }

  public double getAccelerationZ() {
    return m_accelerometer.getZ();
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
    return DriveConstants.diffDriveKinematics.toChassisSpeeds(
        new DifferentialDriveWheelSpeeds(m_leftWheel.getVelocity(), m_rightWheel.getVelocity()));
  }

  public void resetOdometry(Pose2d pose) {
    m_odometry.resetPosition(
        new Rotation2d(m_rotation),
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
    if (Constants.currentMode == Mode.SIM) {
      m_rotation = (m_leftWheel.getPosition() - m_rightWheel.getPosition()) / 2;
    } else {
      m_rotation = m_gyro.getAngle();
    }

    m_leftWheel.periodic();
    m_rightWheel.periodic();
    m_leftWheel.updateInputs();
    m_rightWheel.updateInputs();

    Logger.recordOutput("gyro angle", m_gyro.getAngle());
    Logger.recordOutput("left wheel", m_leftWheel.getPosition());
    Logger.recordOutput("right wheel", m_leftWheel.getPosition());
    m_odometry.update(
        new Rotation2d(m_rotation), m_leftWheel.getPosition(), m_rightWheel.getPosition());
  }
}
