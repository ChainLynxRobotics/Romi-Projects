// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.units.measure.*;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
    public static Mode currentMode = Mode.REAL;
    public static enum Mode {
        REAL,
        REPLAY,
        SIM,
    }

    public static final class AutoConstants {
        public static final double kMaxSpeedMetersPerSecond = 0.3;
        public static final double kMaxAngularSpeedRadiansPerSecond =
            2 * kMaxSpeedMetersPerSecond / DriveConstants.kTrackwidthMeters;
        public static final double kMaxAccelerationMetersPerSecondSquared = 0.3;

        // Reasonable baseline values for a RAMSETE follower in units of meters and seconds
        public static final double kRamseteB = 2;
        public static final double kRamseteZeta = 0.7;
    }

    public final class DriveConstants {
        public static final double kDefaultDriveSpeed = 0.2; // % of max power in volts, 0-1
        public static final double kDefaultRotSpeed = 0.5; // // % of max power in volts, 0-1
        public static final double kCountsPerRevolution = 1440.0;
        public static final Distance kWheelDiameter = Millimeters.of(70); // 70 mm

        public static final Distance translationTolerance = Inches.of(1);
        public static final Angle rotationTolerance = Rotations.of(0.05);

        public static final double kWheelDiameterMeters = 0.07;
        public static final double minVoltage = 2.5;
        public static final double maxVoltage = 10.8;
    
        public static final double ksVolts = 0.929;
        public static final double kvVoltSecondsPerMeter = 6.33;
        public static final double kaVoltSecondsSquaredPerMeter = 0.0389;
    
        public static final double kPDriveVel = 0.085;
    
        public static final double kTrackwidthMeters = 0.142072613;

        public static final double motorPosStdDev = 0.05;
        public static final double motorVelStdDev = 0.01;
    
        public static final DifferentialDriveKinematics diffDriveKinematics =
            new DifferentialDriveKinematics(kTrackwidthMeters);
    }
}   
