package frc.robot.subsystems.Drive;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import frc.robot.Constants.DriveConstants;

public class WheelIOSpark implements WheelIO {

  private final Spark motor;
  private final Encoder encoder;

  public WheelIOSpark(
      int motorChannel, int encoderAChannel, int encoderBChannel, boolean setInverted) {
    motor = new Spark(motorChannel);
    encoder = new Encoder(encoderAChannel, encoderBChannel);

    motor.setInverted(setInverted);
    encoder.setDistancePerPulse(
        (Math.PI * DriveConstants.kWheelDiameterMeters) / DriveConstants.kCountsPerRevolution);
    resetEncoder();
  }

  @Override
  public void updateInputs(WheelIOInputs inputs) {
    inputs.driveAppliedVolts = (motor.get() / 256) * 5;
    inputs.drivePositionMeters = encoder.getDistance();
    inputs.driveVelocityMetersPerSec = encoder.getRate();
  }

  @Override
  public void setDriveVoltage(double volts) {
    motor.setVoltage(Math.max(DriveConstants.minVoltage, Math.min(volts, DriveConstants.maxVoltage)));
  }

  @Override
  public void setDriveSpeed(double speed) {
    motor.set(Math.max(-1, Math.min(speed, 1)));
  }

  @Override
  public void resetEncoder() {
    encoder.reset();
  }
}
