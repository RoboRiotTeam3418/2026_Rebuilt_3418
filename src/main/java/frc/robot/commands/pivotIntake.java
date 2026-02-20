// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

/*

DO NOT RUN THIS CODE, IT IS UNFINSIHED AND COULD POSSIBLY DAMAGE THE ROBOT!!!
DO NOT RUN THIS CODE, IT IS UNFINSIHED AND COULD POSSIBLY DAMAGE THE ROBOT!!!
DO NOT RUN THIS CODE, IT IS UNFINSIHED AND COULD POSSIBLY DAMAGE THE ROBOT!!!

*/

package frc.robot.commands;

//import swervelib.SwerveDrive;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.intakeSubsystem;


/** An example command that uses an example subsystem. */
public class pivotIntake extends Command {
  @SuppressWarnings({ "PMD.UnusedPrivateField", "PMD.SingularField" })
  private final intakeSubsystem m_Intake; // Subsystem

  public pivotIntake(intakeSubsystem intake) { // Constructor | Creates new intakeSubsystem Command
    this.m_Intake = intake;
    addRequirements(intake); // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (m_Intake.ThroughboreEncoder.getPosition() > m_Intake.MAX_ANGLE_IN && m_Intake.ThroughboreEncoder.getPosition() < m_Intake.MAX_ANGLE_OUT){ // Soft stop
      if (m_Intake.ThroughboreEncoder.getPosition() <= m_Intake.MAX_ANGLE_IN + 10 || m_Intake.ThroughboreEncoder.getPosition() <= m_Intake.MAX_ANGLE_OUT - 10){
        m_Intake.pivotMotor.set(m_Intake.pivotSpeed/2); // Speed is halved when getting closer to max limits
      } else {
        m_Intake.pivotMotor.set(m_Intake.pivotSpeed); // If all checks are passed, move normally
      }
    } else {
      m_Intake.pivotMotor.set(0); // If first check fails, don't do anything
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    //m_Intake.pivotMotor.set(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return true; // just in case this code is ran (it isn't supposed to be run yet)
  }
}