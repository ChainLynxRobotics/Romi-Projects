// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.sim;

import static frc.robot.Constants.kUseRealRomi;

import java.util.ArrayList;
import java.util.List;
import org.wpilib.driverstation.RobotState;
import org.wpilib.hardware.hal.SimDouble;
import org.wpilib.math.geometry.Pose2d;
import org.wpilib.math.geometry.Rotation2d;
import org.wpilib.math.util.Units;
import org.wpilib.simulation.EncoderSim;
import org.wpilib.simulation.SimDeviceSim;
import org.wpilib.smartdashboard.Field2d;
import org.wpilib.telemetry.Telemetry;

/**
 * A pretend Romi for the sim window. You do NOT need to read or change this file.
 *
 * <p>It reads the motor outputs, pretends to drive, and draws the pretend Romi on "Sim Field", with
 * a trail showing where it drove (the trail clears every time you enable).
 *
 * <p>No Romi ({@code useRealRomi = false} in build.gradle): it also writes fake encoder and gyro
 * readings, so your code works the same as on a real Romi.
 *
 * <p>Real Romi ({@code useRealRomi = true}): it becomes a "ghost". It still shows where the sim
 * PREDICTS the Romi should be, but leaves the real sensors alone. Compare it with the real Romi!
 */
public class RomiSimulator {
  private static final double kMaxSpeedInchesPerSec = 24.0; // full power
  private static final double kTrackWidthInches = 5.55; // distance between the wheels
  private static final double kMotorLagSeconds = 0.1; // how long the Romi takes to speed up
  private static final double kDeadband = 0.05; // motor power too small to move the Romi
  private static final double kDt = 0.02;
  private static final Pose2d kStartPose = new Pose2d(0.5, 0.5, Rotation2d.ZERO);

  private final Field2d field = new Field2d();
  private final List<Pose2d> trail = new ArrayList<>();
  private boolean wasEnabled;
  private int loopCount;

  private SimDouble leftMotor; // motor power, -1 to 1 (RomiMotor's "speed")
  private SimDouble rightMotor;
  private EncoderSim leftEncoder;
  private EncoderSim rightEncoder;
  private SimDouble gyroAngleZ;

  private double leftSpeed; // inches per second
  private double rightSpeed;
  private double leftDistance; // inches
  private double rightDistance;
  private double headingDegrees;
  private double xMeters = kStartPose.getX();
  private double yMeters = kStartPose.getY();

  /** Call every 20 ms from Robot.simulationPeriodic(). */
  public void update() {
    findDevices();

    double leftPower = 0;
    double rightPower = 0;
    if (RobotState.isEnabled() && leftMotor != null && rightMotor != null) {
      leftPower = applyDeadband(leftMotor.get());
      rightPower = -applyDeadband(rightMotor.get()); // right motor is inverted
    }

    // Speed changes smoothly (the Romi can't change speed instantly)
    leftSpeed += (leftPower * kMaxSpeedInchesPerSec - leftSpeed) * kDt / kMotorLagSeconds;
    rightSpeed += (rightPower * kMaxSpeedInchesPerSec - rightSpeed) * kDt / kMotorLagSeconds;

    // Your code called resetEncoders(), so start counting from 0 again
    if (!kUseRealRomi && leftEncoder != null && leftEncoder.getReset()) {
      leftDistance = 0;
      leftEncoder.setReset(false);
    }
    if (!kUseRealRomi && rightEncoder != null && rightEncoder.getReset()) {
      rightDistance = 0;
      rightEncoder.setReset(false);
    }

    leftDistance += leftSpeed * kDt;
    rightDistance += rightSpeed * kDt;

    // Counter-clockwise is positive
    double turnRateDegrees = Math.toDegrees((rightSpeed - leftSpeed) / kTrackWidthInches);
    headingDegrees += turnRateDegrees * kDt;

    double forwardMeters = Units.inchesToMeters((leftSpeed + rightSpeed) / 2.0) * kDt;
    xMeters += forwardMeters * Math.cos(Math.toRadians(headingDegrees));
    yMeters += forwardMeters * Math.sin(Math.toRadians(headingDegrees));

    // With a real Romi, the real encoders and gyro are used. Don't overwrite them!
    if (!kUseRealRomi) {
      if (leftEncoder != null) {
        leftEncoder.setDistance(leftDistance);
        leftEncoder.setRate(leftSpeed);
      }
      if (rightEncoder != null) {
        rightEncoder.setDistance(rightDistance);
        rightEncoder.setRate(rightSpeed);
      }
      if (gyroAngleZ != null) {
        // A real Romi's gyro counts clockwise as positive, so the pretend one does too
        gyroAngleZ.set(-headingDegrees);
      }
    }

    Pose2d pose = new Pose2d(xMeters, yMeters, Rotation2d.fromDegrees(headingDegrees));
    field.setRobotPose(pose);
    updateTrail(pose);
    Telemetry.log("Sim Field", field);
  }

  private void updateTrail(Pose2d pose) {
    boolean enabled = RobotState.isEnabled();
    if (enabled && !wasEnabled) {
      trail.clear();
    }
    wasEnabled = enabled;
    if (enabled && loopCount++ % 5 == 0) {
      trail.add(pose);
      if (trail.size() > 1000) {
        trail.remove(0);
      }
      field.getObject("Trail").setPoses(trail);
    }
  }

  private static double applyDeadband(double power) {
    return Math.abs(power) < kDeadband ? 0 : power;
  }

  /** Hardware only exists after your code creates it, so keep looking until it's found. */
  private void findDevices() {
    if (leftMotor == null) {
      leftMotor = tryCreate(() -> new SimDeviceSim("PWM:0").getDouble("speed"));
    }
    if (rightMotor == null) {
      rightMotor = tryCreate(() -> new SimDeviceSim("PWM:1").getDouble("speed"));
    }
    if (leftEncoder == null) {
      leftEncoder = tryCreate(() -> EncoderSim.createForChannel(4));
    }
    if (rightEncoder == null) {
      rightEncoder = tryCreate(() -> EncoderSim.createForChannel(6));
    }
    if (gyroAngleZ == null) {
      gyroAngleZ = tryCreate(() -> new SimDeviceSim("Gyro:RomiGyro").getDouble("angle_z"));
    }
  }

  private interface Factory<T> {
    T create();
  }

  private static <T> T tryCreate(Factory<T> factory) {
    try {
      return factory.create();
    } catch (RuntimeException e) {
      return null; // not created yet
    }
  }
}
