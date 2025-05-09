package frc.robot.subsystems.Drive;

import org.littletonrobotics.junction.Logger;

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
    wheel.setDriveSpeed(speed);
  }

  /** position in meters */
  public double getPosition() {
    return wheelAutoLogged.drivePositionMeters;
  }

  /** velocity in m/s */
  public double getVelocity() {
    return wheelAutoLogged.driveVelocityMetersPerSec;
  }

  public void resetEncoder() {
    wheel.resetEncoder();
  }
}
