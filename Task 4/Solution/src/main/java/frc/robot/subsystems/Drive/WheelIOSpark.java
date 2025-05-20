package frc.robot.subsystems.Drive;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import frc.robot.Constants.DriveConstants;

import static edu.wpi.first.units.Units.*;
import edu.wpi.first.units.measure.*;

public class WheelIOSpark implements WheelIO {

  private final Spark motor;
  private final Encoder encoder;

  public WheelIOSpark(
      int motorChannel, int encoderAChannel, int encoderBChannel, boolean setInverted) {
    motor = new Spark(motorChannel);
    encoder = new Encoder(encoderAChannel, encoderBChannel);

    motor.setInverted(setInverted);
    encoder.setDistancePerPulse(
        (DriveConstants.kWheelDiameter.times(Math.PI).div(DriveConstants.kCountsPerRevolution).in(Meters)));
    resetEncoder();
  }

  @Override
  public void updateInputs(WheelIOInputs inputs) {
    inputs.appliedOutput = motor.get();
    inputs.drivePosition = Meters.of(encoder.getDistance());
    inputs.driveVelocity = MetersPerSecond.of(encoder.getRate());
  }

  @Override
  public void setDriveOutput(double speed) {
    motor.set(Math.max(-1, Math.min(speed, 1)));
  }

  @Override
  public void resetEncoder() {
    encoder.reset();
  }
}
