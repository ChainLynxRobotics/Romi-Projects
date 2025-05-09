package frc.robot.commands;

import static edu.wpi.first.units.Units.Rotations;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Drive.RomiDrivetrain;

import edu.wpi.first.units.measure.*;

public class TurnCommand extends Command {
    private RomiDrivetrain drive;
    private Angle angle;

    public TurnCommand(RomiDrivetrain drive, Angle angle) {
        this.drive = drive;
        this.angle = angle;

        addRequirements(drive);
    }

    @Override
    public void initialize() {
        drive.resetGyro();
    }

    @Override
    public void execute() {
        drive.arcadeDrive(0, drive.calculateRotOutput(drive.getAngle().in(Rotations), angle.in(Rotations)));
    }

    @Override
    public boolean isFinished() {
        return drive.atRotationSetpoint();
    }

    @Override
    public void end(boolean interrupted) {
        drive.arcadeDrive(0, 0);
    }
}
