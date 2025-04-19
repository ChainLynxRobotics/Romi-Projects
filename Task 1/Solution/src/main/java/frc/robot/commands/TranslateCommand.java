package frc.robot.commands;

import static frc.robot.Constants.DriveConstants.*;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.RomiDrivetrain;

public class TranslateCommand extends Command {
    private RomiDrivetrain drive;
    private double dist;

    public TranslateCommand(RomiDrivetrain drive, double distInches) {
        this.drive = drive;
        this.dist = distInches;

        addRequirements(drive);
    }

    @Override
    public void initialize() {
        drive.resetEncoders();
    }

    @Override
    public void execute() {
        drive.arcadeDrive(kDefaultDriveSpeed * Math.signum(dist), 0);
    }

    @Override
    public boolean isFinished() {
        return Math.abs((drive.getLeftDistanceInch() + drive.getRightDistanceInch()) / 2) >= Math.abs(dist);
    }

    @Override
    public void end(boolean interrupted) {
        drive.arcadeDrive(0, 0);
    }
}
