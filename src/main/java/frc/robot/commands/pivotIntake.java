// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

//import swervelib.SwerveDrive;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Constants.SubsystemConstants;
import frc.robot.subsystems.intakeSubsystem;


/** An example command that uses an example subsystem. */
public class pivotIntake extends Command {
  @SuppressWarnings({ "PMD.UnusedPrivateField", "PMD.SingularField" })
  private final intakeSubsystem m_Intake; // Subsystem

  // I moved these to constants -Darwin
  double spd;

  public pivotIntake(intakeSubsystem intake, double speed) { // Constructor | Creates new intakeSubsystem Command
    this.m_Intake = intake;
    this.spd = speed;
    addRequirements(intake); // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (m_Intake.ThroughboreEncoder.getPosition() >= SubsystemConstants.INTAKE_MAX_ANGLE_IN && spd>0) {
      m_Intake.pivotMotor.set(0); // Motor shouldn't be running once these constants are reached.
    } else if (m_Intake.ThroughboreEncoder.getPosition() <= SubsystemConstants.INTAKE_MAX_ANGLE_OUT && spd<0){
      m_Intake.pivotMotor.set(0);
    } else {
      m_Intake.pivotMotor.set(SubsystemConstants.INTAKE_PIVOT_SPEED);
    }
    System.out.println(m_Intake.ThroughboreEncoder.getPosition());
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_Intake.pivotMotor.set(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}