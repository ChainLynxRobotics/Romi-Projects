// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.mechanisms;

import static frc.robot.Constants.DriveConstants.*;
import static org.wpilib.units.Units.Seconds;

import java.util.function.DoubleSupplier;
import org.wpilib.command3.Command;
import org.wpilib.command3.Mechanism;
import org.wpilib.drive.DifferentialDrive;
import org.wpilib.hardware.rotation.Encoder;
import org.wpilib.romi.RomiMotor;
import org.wpilib.math.controller.PIDController;
import org.wpilib.romi.RomiGyro;
import org.wpilib.telemetry.Telemetry;
import org.wpilib.tunable.Tunables;

/** The Romi's wheels, motors, and sensors. Commands that drive the Romi live in here. */
public class RomiDrivetrain implements Mechanism {
  // The Romi has the left and right motors on PWM channels 0 and 1
  private final RomiMotor leftMotor = new RomiMotor(0);
  private final RomiMotor rightMotor = new RomiMotor(1);

  // The Romi has encoders on DIO pins 4/5 (left) and 6/7 (right)
  private final Encoder leftEncoder = new Encoder(4, 5);
  private final Encoder rightEncoder = new Encoder(6, 7);

  // The gyro measures how far the Romi has turned
  private final RomiGyro gyro = new RomiGyro();

  private final DifferentialDrive diffDrive =
      new DifferentialDrive(leftMotor::setThrottle, rightMotor::setThrottle);

  private final PIDController drivePID = new PIDController(kDriveP, kDriveI, kDriveD);
  private final PIDController turnPID = new PIDController(kTurnP, kTurnI, kTurnD);

  public RomiDrivetrain() {
    leftEncoder.setDistancePerPulse(kInchesPerPulse);
    rightEncoder.setDistancePerPulse(kInchesPerPulse);
    resetEncoders();

    // The right motor is mounted backwards, so flip it
    rightMotor.setInverted(true);

    // When no other command is using the drivetrain, keep the motors stopped
    setDefaultCommand(
        runRepeatedly(this::stop).withPriority(Command.LOWEST_PRIORITY).named("Stopped"));

    drivePID.setTolerance(kDriveToleranceInches, kDriveSpeedToleranceInchesPerSec);
    Tunables.publish("Drive PID", drivePID); // lets you change P, I, D live in the sim window

    turnPID.setTolerance(kTurnToleranceDegrees, kTurnSpeedToleranceDegreesPerSec);
    Tunables.publish("Turn PID", turnPID);
  }

  /**
   * Drives the Romi.
   *
   * @param forwardSpeed -1 (full backward) to 1 (full forward)
   * @param rotationSpeed -1 (full clockwise) to 1 (full counter-clockwise)
   */
  public void arcadeDrive(double forwardSpeed, double rotationSpeed) {
    diffDrive.arcadeDrive(forwardSpeed, rotationSpeed, false);
  }

  public void stop() {
    diffDrive.stopMotor();
  }

  public void resetEncoders() {
    leftEncoder.reset();
    rightEncoder.reset();
  }

  public double getLeftDistanceInches() {
    return leftEncoder.getDistance();
  }

  public double getRightDistanceInches() {
    return rightEncoder.getDistance();
  }

  public double getAverageDistanceInches() {
    return (getLeftDistanceInches() + getRightDistanceInches()) / 2;
  }

  /** Counter-clockwise (turning left) is positive. */
  public double getHeadingDegrees() {
    return -gyro.getAngleZ(); // the Romi's gyro counts clockwise as positive, so flip it
  }

  // ---- Commands ----

  /** Drives with the joysticks. Runs forever (until another command takes over). */
  public Command driveWithJoysticks(DoubleSupplier forward, DoubleSupplier rotation) {
    return runRepeatedly(() -> arcadeDrive(forward.getAsDouble(), rotation.getAsDouble()))
        .named("Joystick Drive");
  }

  /** Drives forward a distance, then stops. */
  public Command driveForward(double inches) {
    return run(coroutine -> {
          double start = getAverageDistanceInches();

          while (getAverageDistanceInches() - start < inches) {
            arcadeDrive(kDefaultDriveSpeed, 0);
            coroutine.yield(); // wait 20 ms, let everything else run, then keep going
          }

          stop();
        })
        .named("Drive Forward");
  }

  /** Drives forward a distance using PID, so it slows down and stops right on target. */
  public Command driveForwardPID(double inches) {
    return run(coroutine -> {
          double target = getAverageDistanceInches() + inches;
          drivePID.reset();
          drivePID.setSetpoint(target);

          while (!drivePID.atSetpoint()) {
            double speed = drivePID.calculate(getAverageDistanceInches());
            speed = Math.clamp(speed, -kMaxDrivePIDSpeed, kMaxDrivePIDSpeed);
            arcadeDrive(speed, 0);

            Telemetry.log("Drive Target (in)", target);
            coroutine.yield();
          }

          stop();
        })
        .named("Drive Forward PID")
        .withTimeout(Seconds.of(3)); // give up after 3 seconds, just in case
  }

  /** Turns left (counter-clockwise) some degrees, then stops. */
  public Command turnLeft(double degrees) {
    return run(coroutine -> {
          double start = getHeadingDegrees();

          while (getHeadingDegrees() - start < degrees - kTurnStopEarlyDegrees) {
            arcadeDrive(0, kDefaultRotSpeed);
            coroutine.yield();
          }

          stop();
        })
        .named("Turn Left");
  }

  /** Turns left (counter-clockwise) some degrees using PID. Negative degrees turn right. */
  public Command turnLeftPID(double degrees) {
    return run(coroutine -> {
          double target = getHeadingDegrees() + degrees;
          turnPID.reset();
          turnPID.setSetpoint(target);

          while (!turnPID.atSetpoint()) {
            double speed = turnPID.calculate(getHeadingDegrees());
            speed = Math.clamp(speed, -kMaxTurnPIDSpeed, kMaxTurnPIDSpeed);
            arcadeDrive(0, speed);

            Telemetry.log("Turn Target (deg)", target);
            coroutine.yield();
          }

          stop();
        })
        .named("Turn Left PID")
        .withTimeout(Seconds.of(3));
  }

  /** Sends numbers to the sim window so you can see them. Robot.java calls this every 20 ms. */
  public void logTelemetry() {
    Telemetry.log("Left Distance (in)", getLeftDistanceInches());
    Telemetry.log("Right Distance (in)", getRightDistanceInches());
    Telemetry.log("Average Distance (in)", getAverageDistanceInches());
    Telemetry.log("Heading (deg)", getHeadingDegrees());
  }
}
