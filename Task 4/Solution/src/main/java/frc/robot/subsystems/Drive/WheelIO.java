package frc.robot.subsystems.Drive;

import org.littletonrobotics.junction.AutoLog;

public interface WheelIO {

  @AutoLog
  class WheelIOInputs {
    public double drivePositionMeters = 0.0;
    public double driveVelocityMetersPerSec = 0.0;
    public double driveAppliedVolts = 0.0;
  }

  /** Updates the set of loggable inputs. */
  public default void updateInputs(WheelIOInputs inputs) {}

  public default void setDriveVoltage(double volts) {}

  public default void setDriveSpeed(double speed) {}

  public default void resetEncoder() {}
}
