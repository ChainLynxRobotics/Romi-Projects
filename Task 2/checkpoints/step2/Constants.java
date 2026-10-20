// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

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
  }
}
