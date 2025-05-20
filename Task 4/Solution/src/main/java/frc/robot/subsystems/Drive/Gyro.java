package frc.robot.subsystems.Drive;

import org.littletonrobotics.junction.Logger;
import static edu.wpi.first.units.Units.*;
import edu.wpi.first.units.measure.*;

public class Gyro {
    private final GyroIO io;
    private final GyroIOInputsAutoLogged inputs = new GyroIOInputsAutoLogged();

    public Gyro(GyroIO gryo) {
        this.io = gryo;
    }

    public void updateInputs() {
        io.updateInputs(inputs);
    }

    public void periodic() {
        Logger.processInputs("Drive/Gryo", inputs);
    }

    public Angle getAngle() {
        return inputs.gyroAngle;
    }

    public AngularVelocity getVelocity() {
        return inputs.gyroVelocity;
    }

    public void resetGyro() {
        io.resetGyro();
    }
}
