// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static frc.robot.Constants.DriveConstants.*;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPLTVController;

import dev.doglog.DogLog;

import static edu.wpi.first.units.Units.*;
import edu.wpi.first.units.measure.*;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelPositions;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj.romi.RomiGyro;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveConstants;

public class RomiDrivetrain extends SubsystemBase {
  // The Romi has the left and right motors set to
  // PWM channels 0 and 1 respectively
  private final Spark leftMotor = new Spark(0);
  private final Spark rightMotor = new Spark(1);

  // The Romi has onboard encoders that are hardcoded
  // to use DIO pins 4/5 and 6/7 for the left and right
  private final Encoder leftEncoder = new Encoder(4, 5);
  private final Encoder rightEncoder = new Encoder(6, 7);

  private final RomiGyro gyro = new RomiGyro();

  // Set up the differential drive controller
  private final DifferentialDrive diffDrive =
      new DifferentialDrive(leftMotor::set, rightMotor::set);

  // PID controllers
  private final PIDController rotController;
  private final PIDController translateController;
  private Angle curRotSetpoint = Radians.zero();
  private Distance curTransSetpoint = Meters.zero();

  private DifferentialDriveOdometry odometry = new DifferentialDriveOdometry(getRotation(), getLeftDistance(), getRightDistance());

  /** Creates a new RomiDrivetrain. */
  public RomiDrivetrain() {
    // Use inches as unit for encoder distances
    leftEncoder.setDistancePerPulse(kWheelDiameter.times(Math.PI).div(kCountsPerRevolution).in(Meters));
    rightEncoder.setDistancePerPulse(kWheelDiameter.times(Math.PI).div(kCountsPerRevolution).in(Meters));
    resetEncoders();
    resetGyro();

    // Invert right side since motor is flipped
    rightMotor.setInverted(true);

    translateController = new PIDController(kTranslationP, kTranslationI, kTranslationD);
    rotController = new PIDController(kRotationP, kRotationI, kRotationD);
    translateController.setTolerance(DriveConstants.kTranslationTolerance.baseUnitMagnitude());
    rotController.setTolerance(DriveConstants.kRotationTolerance.baseUnitMagnitude());

    RobotConfig ppConfig;
    // load config from Path planner GUI
    try {
      ppConfig = RobotConfig.fromGUISettings();
    } catch (Exception e) {
      e.printStackTrace();
      ppConfig = null;
    }

    // Configure AutoBuilder last
    AutoBuilder.configure(
            this::getPose, // Robot pose supplier
            this::resetPose, // Method to reset odometry (will be called if your auto has a starting pose)
            this::getRobotRelitiveSpeeds, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
            (speeds, feedforwards) -> driveRobotRelative(speeds), // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds. Also optionally outputs individual module feedforwards
            new PPLTVController(0.02), // PPLTVController is the built in path following controller for differential drive trains
            ppConfig, // The robot configuration
            () -> {
              // Boolean supplier that controls when the path will be mirrored for the red alliance
              // This will flip the path being followed to the red side of the field.
              // THE ORIGIN WILL REMAIN ON THE BLUE SIDE

              var alliance = DriverStation.getAlliance();
              if (alliance.isPresent()) {
                return alliance.get() == DriverStation.Alliance.Red;
              }
              return false;
            },
            this // Reference to this subsystem to set requirements
    );

  }

  public void arcadeDrive(double xaxisSpeed, double zaxisRotate) {
    diffDrive.arcadeDrive(xaxisSpeed, zaxisRotate);
  }

  public void driveRobotRelative(ChassisSpeeds speeds) {
    arcadeDrive(MetersPerSecond.of(speeds.vxMetersPerSecond).div(kMaxSpeed).baseUnitMagnitude(), RadiansPerSecond.of(speeds.omegaRadiansPerSecond).div(kMaxAngularVelocity).baseUnitMagnitude());
  }

  public Pose2d getPose() {
    return odometry.update(getRotation(), new DifferentialDriveWheelPositions(getLeftDistance(), getRightDistance()));
  }

  private void resetPose(Pose2d pose) {
    odometry.resetPose(pose);
  }

  public ChassisSpeeds getRobotRelitiveSpeeds() {
    return new ChassisSpeeds(getAverageVelocity(), MetersPerSecond.zero(), getAngularVelocity());
  }
  
  public void resetEncoders() {
    leftEncoder.reset();
    rightEncoder.reset();
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

  public LinearVelocity getAverageVelocity() {
    return MetersPerSecond.of((leftEncoder.getRate() + rightEncoder.getRate()) / 2);
  }

  /** Gyro angle in degrees */
  public Angle getAngle() {
    return Degrees.of(gyro.getAngle());
  }

  public AngularVelocity getAngularVelocity() {
    return DegreesPerSecond.of(gyro.getRate());
  }

  private Rotation2d getRotation() {
    return new Rotation2d(getAngle());
  }

  public void resetGyro() {
    gyro.reset();
  }

  public double calculateRotOutput(Angle setpoint) {
    curRotSetpoint = setpoint;
    return rotController.calculate(getAngle().baseUnitMagnitude(), setpoint.baseUnitMagnitude());
  }

  public double calculateTranslateOutput(Distance setpoint) {
    curTransSetpoint = setpoint;
    return translateController.calculate(getAverageDistance().baseUnitMagnitude(), setpoint.baseUnitMagnitude());
  }

  public boolean atTranslationSetpoint() {
    return translateController.atSetpoint();
  }

  public boolean atRotationSetpoint() {
    return rotController.atSetpoint();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    DogLog.log("drivetrain/position meters", getAverageDistance().in(Meters));
    DogLog.log("drivetrain/rotation degrees", getAngle().in(Degrees));
    DogLog.log("drivetrain/translation setpoint", curTransSetpoint.in(Meters));
    DogLog.log("drivetrain/rotation setpoint", curRotSetpoint.in(Degrees));
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
