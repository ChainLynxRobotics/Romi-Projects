// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.mechanisms.RomiDrivetrain;
import frc.robot.sim.RomiSimulator;
import org.wpilib.command3.Scheduler;
import org.wpilib.framework.OpModeRobot;
import org.wpilib.system.DataLogManager;

/**
 * The Robot holds everything the robot is made of (mechanisms, controllers).
 *
 * <p>OpModes (Teleop and Autonomous classes) get this Robot passed to them, so they can use
 * everything in here.
 */
public class Robot extends OpModeRobot {
  // ---- Mechanisms ----
  public final RomiDrivetrain drivetrain = new RomiDrivetrain();

  // ---- Simulator (don't change) ----
  private final RomiSimulator simulator = new RomiSimulator();

  public Robot() {
    // Record every run to a log file in the "logs" folder, so you can look at it later
    DataLogManager.start();
  }

  /** Runs every 20 ms, no matter what mode the robot is in. */
  @Override
  public void robotPeriodic() {
    // Runs all the commands. Without this line, nothing happens!
    Scheduler.getDefault().run();

    drivetrain.logTelemetry();
  }

  /** Runs once every time the robot is disabled. */
  @Override
  public void disabledInit() {
    // Stop every command when the robot is disabled.
    Scheduler.getDefault().cancelAll();
  }

  /** Runs every 20 ms in simulation only. */
  @Override
  public void simulationPeriodic() {
    simulator.update();
  }
}
