package frc.robot.subsystems.Drive;

import org.littletonrobotics.junction.AutoLog;

import static edu.wpi.first.units.Units.*;
import edu.wpi.first.units.measure.*;

public interface GyroIO {
    
    @AutoLog
    class GyroIOInputs {
        public Angle gyroAngle = Rotations.zero();
        public AngularVelocity gyroVelocity = RotationsPerSecond.zero();
    }

    public default void updateInputs(GyroIOInputs inputs) {}

    public default void resetGyro() {}
}  
