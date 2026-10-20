// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.mechanisms;

import static frc.robot.Constants.DriveConstants.*;

import java.util.function.DoubleSupplier;
import org.wpilib.command3.Command;
import org.wpilib.command3.Mechanism;
import org.wpilib.drive.DifferentialDrive;
import org.wpilib.hardware.rotation.Encoder;
import org.wpilib.romi.RomiMotor;
import org.wpilib.telemetry.Telemetry;

/** The Romi's wheels, motors, and sensors. Commands that drive the Romi live in here. */
public class RomiDrivetrain implements Mechanism {
  // The Romi has the left and right motors on PWM channels 0 and 1
  private final RomiMotor leftMotor = new RomiMotor(0);
  private final RomiMotor rightMotor = new RomiMotor(1);

  // The Romi has encoders on DIO pins 4/5 (left) and 6/7 (right)
  private final Encoder leftEncoder = new Encoder(4, 5);
  private final Encoder rightEncoder = new Encoder(6, 7);

  private final DifferentialDrive diffDrive =
      new DifferentialDrive(leftMotor::setThrottle, rightMotor::setThrottle);

  public RomiDrivetrain() {
    leftEncoder.setDistancePerPulse(kInchesPerPulse);
    rightEncoder.setDistancePerPulse(kInchesPerPulse);
    resetEncoders();

    // The right motor is mounted backwards, so flip it
    rightMotor.setInverted(true);

    // When no other command is using the drivetrain, keep the motors stopped
    setDefaultCommand(
        runRepeatedly(this::stop).withPriority(Command.LOWEST_PRIORITY).named("Stopped"));
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

  // ---- Commands ----

  /** Drives with the joysticks. Runs forever (until another command takes over). */
  public Command driveWithJoysticks(DoubleSupplier forward, DoubleSupplier rotation) {
    return runRepeatedly(() -> arcadeDrive(forward.getAsDouble(), rotation.getAsDouble()))
        .named("Joystick Drive");
  }

  /** Sends numbers to the sim window so you can see them. Robot.java calls this every 20 ms. */
  public void logTelemetry() {
    Telemetry.log("Left Distance (in)", getLeftDistanceInches());
    Telemetry.log("Right Distance (in)", getRightDistanceInches());
  }
}
