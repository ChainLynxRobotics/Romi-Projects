package frc.robot.subsystems.Drive;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import frc.robot.Constants.DriveConstants;
import static frc.robot.Constants.DriveConstants.*;

import static edu.wpi.first.units.Units.*;
import edu.wpi.first.units.measure.*;

public class WheelIOSim implements WheelIO {
  private static final double LOOP_PERIOD_SECS = 0.02;

  private DCMotorSim driveSim = new DCMotorSim(
    LinearSystemId.createDCMotorSystem(
      DCMotor.getKrakenX60(2), 0, 0), 
      DCMotor.getKrakenX60Foc(2), 
      DriveConstants.kMotorPosStdDev, DriveConstants.kMotorVelStdDev);
  private double driveAppliedOutput = 0.0;

  @Override
  public void updateInputs(WheelIOInputs inputs) {
    driveSim.update(LOOP_PERIOD_SECS);

    inputs.drivePosition =
        (Distance) kAngleToDist.timesDivisor(Rotations.of(driveSim.getAngularPositionRotations()));
    inputs.driveVelocity =
        (LinearVelocity) MetersPerSecond.of(kAngleToDist.timesDivisor(Rotations.of(driveSim.getAngularVelocityRPM())).in(Meters));
    inputs.appliedOutput = driveAppliedOutput;
  }

  @Override
  public void setDriveOutput(double output) {
    driveAppliedOutput = MathUtil.clamp(output, -1, 1);
    driveSim.setInput(driveAppliedOutput);
  }
}
