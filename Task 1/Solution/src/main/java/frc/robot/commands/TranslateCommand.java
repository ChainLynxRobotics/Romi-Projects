package frc.robot.commands;

import static frc.robot.Constants.DriveConstants.*;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.RomiDrivetrain;

import edu.wpi.first.units.measure.*;

public class TranslateCommand extends Command {
    private RomiDrivetrain drive;
    private Distance dist;
    // should be 1 or -1
    private double dir;

    public TranslateCommand(RomiDrivetrain drive, Distance dist) {
        this.drive = drive;
        this.dist = dist;

        this.dir = Math.signum(this.dist.baseUnitMagnitude());

        addRequirements(drive);
    }

    @Override
    public void initialize() {
        drive.resetEncoders();
    }

    @Override
    public void execute() {
        drive.arcadeDrive(kDefaultDriveSpeed * dir, 0);
    }

    @Override
    public boolean isFinished() {
        return drive.getAverageDistance().times(dir).gte(dist);
    }

    @Override
    public void end(boolean interrupted) {
        drive.arcadeDrive(0, 0);
    }
}
