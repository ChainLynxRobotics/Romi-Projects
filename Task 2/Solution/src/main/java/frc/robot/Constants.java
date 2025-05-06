// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;
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
    public final class DriveConstants {
        public static final double kDefaultDriveSpeed = 0.2; // % of max power in volts, 0-1
        public static final double kDefaultRotSpeed = 0.5; // // % of max power in volts, 0-1
        public static final double kCountsPerRevolution = 1440.0;
        public static final Distance kWheelDiameter = Millimeters.of(70); // 70 mm

        public static final Distance kTranslationTolerance = Inches.of(1);
        public static final Angle kRotationTolerance = Rotations.of(0.05);

        public static final double kTranslationP = 0.1;
        public static final double kTranslationI = 0;
        public static final double kTranslationD = 0;

        public static final double kRotationP = 10;
        public static final double kRotationI  = 0;
        public static final double kRotationD  = 0;
    }
}   
