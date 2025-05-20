package frc.robot.subsystems.Drive;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.DegreesPerSecond;

import edu.wpi.first.wpilibj.romi.RomiGyro;

public class GyroIORomi implements GyroIO {
    
    private final RomiGyro gyro;

    public GyroIORomi() {
        gyro = new RomiGyro();
        resetGyro();
    }

    @Override
    public void updateInputs(GyroIOInputs inputs) {
        inputs.gyroAngle = Degrees.of(gyro.getAngle());
        inputs.gyroVelocity = DegreesPerSecond.of(gyro.getRate());
    }

    @Override
    public void resetGyro() {
        gyro.reset();
    }
}
