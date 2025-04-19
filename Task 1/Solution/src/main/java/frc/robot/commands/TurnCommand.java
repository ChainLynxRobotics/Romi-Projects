package frc.robot.commands;

import static frc.robot.Constants.DriveConstants.*;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.RomiDrivetrain;

public class TurnCommand extends Command {
    private RomiDrivetrain drive;
    private double angle;

    public TurnCommand(RomiDrivetrain drive, double angle) {
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
        drive.arcadeDrive(0, kDefaultRotSpeed * Math.signum(angle));
    }

    @Override
    public boolean isFinished() {
        return Math.abs(drive.getAngle()) >= Math.abs(angle);
    }

    @Override
    public void end(boolean interrupted) {
        drive.arcadeDrive(0, 0);
    }
}
