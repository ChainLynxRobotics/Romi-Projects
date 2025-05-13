package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.RomiDrivetrain;

import static edu.wpi.first.units.Units.*;
import edu.wpi.first.units.measure.*;

public class TranslateCommand extends Command {
    private RomiDrivetrain drive;
    private Distance dist;

    public TranslateCommand(RomiDrivetrain drive, Distance dist) {
        this.drive = drive;
        this.dist = dist;

        addRequirements(drive);
    }

    @Override
    public void initialize() {
        drive.resetEncoders();
    }

    @Override
    public void execute() {
        drive.arcadeDrive(drive.calculateTranslateOutput(dist), 0);
    }

    @Override
    public boolean isFinished() {
        return drive.atTranslationSetpoint();
    }

    @Override
    public void end(boolean interrupted) {
        drive.arcadeDrive(0, 0);
    }
}
