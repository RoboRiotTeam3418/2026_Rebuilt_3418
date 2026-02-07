// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Climber;

/** An example command that uses an example subsystem. */
public class ClimbCmd extends Command {
  @SuppressWarnings({ "PMD.UnusedPrivateField", "PMD.SingularField" })
  private final Climber climb;
  private double height;
  private PIDController pid;
  @SuppressWarnings("unused")
  private final static double ALLOWANCE = 1; // inches
  private final static double CLIMB_P = .05, CLIMB_I = 0.0025, CLIMB_D = 0.00;

  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public ClimbCmd(Climber climbSubsystem, double toHeight) { // Sets everything up
    this.climb=climbSubsystem;
    this.height=toHeight;
    addRequirements(climbSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    pid = new PIDController(CLIMB_P, CLIMB_I, CLIMB_D);
    pid.setTolerance(2, 5);// values suggested by wpilib documentation
    pid.setSetpoint(height);

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    climb.climb(pid.calculate(climb.getCurrentHeight()));
    
  }
}