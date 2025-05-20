package frc.robot.subsystems.Drive;

import org.littletonrobotics.junction.AutoLog;

import static edu.wpi.first.units.Units.*;
import edu.wpi.first.units.measure.*;

public interface WheelIO {

  @AutoLog
  class WheelIOInputs {
    public Distance drivePosition = Meters.zero();
    public LinearVelocity driveVelocity = MetersPerSecond.zero();
    public double appliedOutput;
  }

  /** Updates the set of loggable inputs. */
  public default void updateInputs(WheelIOInputs inputs) {}

  public default void setDriveOutput(double output) {}

  public default void resetEncoder() {}
}
