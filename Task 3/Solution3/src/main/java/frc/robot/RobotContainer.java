// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;
import edu.wpi.first.units.measure.*;

import static edu.wpi.first.wpilibj2.command.Commands.*;

import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;

import dev.doglog.DogLog;
import dev.doglog.DogLogOptions;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.commands.DriveCommand;
import frc.robot.commands.TranslateCommand;
import frc.robot.commands.TurnCommand;
import frc.robot.subsystems.RomiDrivetrain;
import edu.wpi.first.wpilibj2.command.Command;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final RomiDrivetrain romiDrivetrain;

  private final Distance distToDrive;
  private final Angle angleToTurn;

  private final DriveCommand driveCommand;
  private final Command driveCommand2;
  private final TranslateCommand translateCommand;
  private final Command translateCommand2;
  private final TurnCommand turnCommand;
  private final Command turnCommand2;

  private final PathPlannerAuto driveAuto;
  private final PathPlannerAuto curveAuto;
  private final PathPlannerAuto finalAuto;

  private final Joystick joystick = new Joystick(0);
  private final SendableChooser<Command> autoChooser = new SendableChooser<>();

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    DogLog.setOptions(new DogLogOptions().withCaptureNt(true));
    
    romiDrivetrain = new RomiDrivetrain();

    distToDrive = Inches.of(6);
    angleToTurn = Rotations.of(0.5);

    driveCommand = new DriveCommand(romiDrivetrain, () -> joystick.getY(), () -> joystick.getX());
    driveCommand2 = runEnd(() -> romiDrivetrain.arcadeDrive(joystick.getY(), joystick.getX()), () -> romiDrivetrain.arcadeDrive(0, 0), romiDrivetrain);
    translateCommand = new TranslateCommand(romiDrivetrain, distToDrive);
    translateCommand2 = runEnd(() -> romiDrivetrain.arcadeDrive(romiDrivetrain.calculateTranslateOutput(distToDrive), 0), () -> romiDrivetrain.arcadeDrive(0, 0), romiDrivetrain).until(romiDrivetrain::atTranslationSetpoint).beforeStarting(runOnce(romiDrivetrain::resetEncoders));
    turnCommand = new TurnCommand(romiDrivetrain, angleToTurn);
    turnCommand2 = runEnd(() -> romiDrivetrain.arcadeDrive(0, romiDrivetrain.calculateRotOutput(angleToTurn)), () -> romiDrivetrain.arcadeDrive(0, 0), romiDrivetrain).until(romiDrivetrain::atRotationSetpoint).beforeStarting(runOnce(romiDrivetrain::resetGyro));

    NamedCommands.registerCommand("Turn 360", sequence(turnCommand, turnCommand2));

    driveAuto = new PathPlannerAuto("drive");
    curveAuto = new PathPlannerAuto("curve");
    finalAuto = new PathPlannerAuto("drive spin turn");
    
    romiDrivetrain.setDefaultCommand(driveCommand);

    autoChooser.addOption("drive 6 inches", translateCommand);
    autoChooser.addOption("drive 6 inches comp", translateCommand2);
    autoChooser.addOption("turn 180", turnCommand);
    autoChooser.addOption("turn 180 comp", turnCommand2);

    autoChooser.addOption("drive auto", driveAuto);
    autoChooser.addOption("curve auto", curveAuto);
    autoChooser.addOption("final auto", finalAuto);

    SmartDashboard.putData(autoChooser);
    // Configure the button bindings
    configureButtonBindings();
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
    // An ExampleCommand will run in autonomous
    return autoChooser.getSelected();
  }
}
