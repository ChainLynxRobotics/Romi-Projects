package frc.robot.subsystems.Drive;

import org.littletonrobotics.junction.Logger;

import static edu.wpi.first.units.Units.*;
import edu.wpi.first.units.measure.*;

public class Wheel { 
  private final int index;
  private final WheelIO wheel;
  private final WheelIOInputsAutoLogged wheelAutoLogged = new WheelIOInputsAutoLogged();

  public Wheel(WheelIO wheel, int index) {
    this.wheel = wheel;
    this.index = index;
  }

  public void updateInputs() {
    wheel.updateInputs(wheelAutoLogged);
  }

  public void periodic() {
    Logger.processInputs("Drive/Wheel" + Integer.toString(index), wheelAutoLogged);
  }

  public void set(double speed) {
    System.out.println("setting wheel " + Integer.toString(index) + " speed to: " + speed);
    wheel.setDriveOutput(speed);
  }

  /** position in meters */
  public Distance getPosition() {
    return wheelAutoLogged.drivePosition;
  }

  /** velocity in m/s */
  public LinearVelocity getVelocity() {
    return wheelAutoLogged.driveVelocity;
  }

  public void resetEncoder() {
    wheel.resetEncoder();
  }
}
