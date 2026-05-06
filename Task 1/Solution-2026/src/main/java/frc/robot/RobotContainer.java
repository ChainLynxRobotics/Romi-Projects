// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.units.measure.*;

import static edu.wpi.first.wpilibj2.command.Commands.*;
import static frc.robot.Constants.DriveConstants.*;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.RomiDrivetrain;
import static edu.wpi.first.wpilibj2.command.Commands.*;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import static frc.robot.Constants.DriveConstants.*;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final RomiDrivetrain romiDrivetrain;

  private final SendableChooser<Command> autoChooser = new SendableChooser<>();

  private final CommandXboxController driveController = new CommandXboxController(0);
  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    romiDrivetrain = new RomiDrivetrain();
    SmartDashboard.putData(autoChooser);

    romiDrivetrain.setDefaultCommand(driveCommand());

    driveController
      .a()
      .onTrue(rotateCommand());

    driveController
      .b()
      .onTrue(translateCommand());
    // Configure the button bindings
    configureButtonBindings();
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link edu.wpi.first.wpilibj.GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.driveController} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.driveControllerButton}.
   */
  private void configureButtonBindings() {}

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An ExampleCommand will run in autonomous
    return autoChooser.getSelected();
  }

  public Command driveCommand() {
    return run(() -> romiDrivetrain.arcadeDrive(-driveController.getLeftY(), -driveController.getRightX()), romiDrivetrain);
  }

  public Command translateCommand() {
    return runOnce(() -> romiDrivetrain.resetEncoders())
      .andThen(run(() -> romiDrivetrain.arcadeDrive(-kDefaultDriveSpeed * 1, 0)))
      .until(() -> romiDrivetrain.getAverageDistance().lte(Inches.of(-5)));
  }

  public Command rotateCommand() {
    return runOnce(() -> romiDrivetrain.resetGyro())
      .andThen(run(() -> romiDrivetrain.arcadeDrive(0, -kDefaultRotSpeed * 1)))
      .until(() -> romiDrivetrain.getAngle().gte(Degrees.of(45)));
  }
}