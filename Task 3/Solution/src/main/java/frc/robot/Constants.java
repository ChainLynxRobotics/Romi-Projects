// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import org.wpilib.math.geometry.Pose2d;
import org.wpilib.math.geometry.Rotation2d;

/** Numbers the whole robot uses. Nothing functional goes in here. */
public final class Constants {
  /**
   * Using a real Romi? Don't change this here: set {@code useRealRomi} at the top of build.gradle.
   */
  public static final boolean kUseRealRomi = "true".equals(System.getenv("USE_REAL_ROMI"));

  public static final class DriveConstants {
    public static final double kCountsPerRevolution = 1440.0;
    public static final double kWheelDiameterInches = 2.75591; // 70 mm
    public static final double kInchesPerPulse =
        Math.PI * kWheelDiameterInches / kCountsPerRevolution;

    public static final double kDefaultDriveSpeed = 0.4; // fraction of max power, 0 to 1
    public static final double kDefaultRotSpeed = 0.2; // fraction of max power, 0 to 1

    // Turns overshoot because of delay. Measure: (how much Heading went up) - 90
    public static final double kTurnStopEarlyDegrees = 8;

    // Drive PID
    public static final double kDriveP = 0.2;
    public static final double kDriveI = 0;
    public static final double kDriveD = 0;
    public static final double kDriveToleranceInches = 0.5; // "close enough"
    public static final double kDriveSpeedToleranceInchesPerSec = 1; // "almost stopped"
    public static final double kMaxDrivePIDSpeed = 0.6;

    // Turn PID
    public static final double kTurnP = 0.03;
    public static final double kTurnI = 0;
    public static final double kTurnD = 0.002; // a little D stops the turn from wobbling
    public static final double kTurnToleranceDegrees = 2;
    public static final double kTurnSpeedToleranceDegreesPerSec = 5;
    public static final double kMaxTurnPIDSpeed = 0.3;

    // Odometry
    public static final Pose2d kStartPose = new Pose2d(0.5, 0.5, Rotation2d.ZERO); // meters

    // Paths
    public static final double kTrackWidthMeters = 0.141; // distance between the wheels
    public static final double kMaxPathSpeedMetersPerSec = 0.3;
    public static final double kMaxPathAccelMetersPerSecSq = 0.3;
    public static final double kMaxSpeedMetersPerSec = 0.6; // Romi top speed at full power
  }
}
