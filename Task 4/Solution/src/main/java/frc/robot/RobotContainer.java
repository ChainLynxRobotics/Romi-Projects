// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.wpilibj2.command.Commands.*;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.commands.DriveCommand;
import frc.robot.commands.TurnCommand;
import frc.robot.subsystems.Drive.GyroIO;
import frc.robot.subsystems.Drive.GyroIORomi;
import frc.robot.subsystems.Drive.GyroIOSim;
import frc.robot.subsystems.Drive.RomiDrivetrain;
import frc.robot.subsystems.Drive.WheelIO;
import frc.robot.subsystems.Drive.WheelIOSim;
import frc.robot.subsystems.Drive.WheelIOSpark;

import static edu.wpi.first.units.Units.*;
import edu.wpi.first.units.measure.*;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final RomiDrivetrain drivetrain;
  private final Joystick controller = new Joystick(0);

  private final Command autoCommand;
  private final SendableChooser<Command> autoChooser;

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    switch (Constants.currentMode) {
      case REAL:
        drivetrain =
            new RomiDrivetrain(new WheelIOSpark(0, 4, 5, false), new WheelIOSpark(1, 6, 7, true), new GyroIORomi());
        break;
      case SIM:
        drivetrain = new RomiDrivetrain(new WheelIOSim(), new WheelIOSim(), new GyroIOSim());
        break;
      default:
        drivetrain = new RomiDrivetrain(new WheelIO() {}, new WheelIO() {}, new GyroIO() {});
    }

    autoCommand = print("placeholder");

    // Configure the button bindings
    configureButtonBindings();
    drivetrain.setDefaultCommand(new DriveCommand(drivetrain, () -> controller.getY(), () -> controller.getX()));

    autoChooser = new SendableChooser<>();
    autoChooser.setDefaultOption("simple drive", autoCommand);
    autoChooser.addOption(
        "multi drive", schedulePath(new Command[] {translateRomi(0.5, 0.5), rotateRomi(180, 10)}));
    TurnCommand turnCommand = new TurnCommand(drivetrain, Degrees.of(50));
    autoChooser.addOption("rotate romi command", turnCommand);
    SmartDashboard.putData("autoChooser", autoChooser);
  }

  public Command rotateRomi(double degrees, double threshold) {
    double initSetpoint = drivetrain.getAngle().in(Degrees) + degrees;
    if (Math.signum(drivetrain.getAngle().in(Degrees)) != Math.signum(initSetpoint)) {
      initSetpoint += -Math.signum(initSetpoint) * 360;
    }
    double setpoint = initSetpoint;

    return run(
            () -> {
              double factor = Math.signum(setpoint) * Math.signum(drivetrain.getAngle().in(Degrees));
              double output =
                  drivetrain.calculateRotOutput(drivetrain.getAngle().in(Degrees) * factor, setpoint);
              SmartDashboard.putNumber("rotPidOutput", output);
              drivetrain.arcadeDrive(0, output);
            },
            drivetrain)
        .until(
            () ->
                Math.abs(
                        Math.signum(setpoint)
                                * Math.signum(drivetrain.getAngle().in(Degrees))
                                * drivetrain.getAngle().in(Degrees)
                            - setpoint)
                    < threshold)
        .finallyDo(() -> drivetrain.arcadeDrive(0, 0));
  }

  public Command translateRomi(double distMeters, double threshold) {
    double setpoint = drivetrain.getLeftDistance().in(Meters) + distMeters;
    return run(
            () -> {
              double output =
                  drivetrain.calculateTranslateOutput(
                      drivetrain.getLeftDistance().in(Meters), setpoint);
              SmartDashboard.putNumber("translationPidOutput", output);
              drivetrain.arcadeDrive(output, 0);
            },
            drivetrain)
        .until(() -> Math.abs(drivetrain.getLeftDistance().in(Meters) - setpoint) < threshold)
        .finallyDo(() -> drivetrain.arcadeDrive(0, 0));
  }

  public Command schedulePath(Command[] pathCommands) {
    return sequence(pathCommands);
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link edu.wpi.first.wpilibj.GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {}

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return autoChooser.getSelected();
  }
}
