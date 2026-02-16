// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

/*

DO NOT RUN THIS CODE, IT IS UNFINSIHED AND WILL DESTROY THE INTAKE!!!
DO NOT RUN THIS CODE, IT IS UNFINSIHED AND WILL DESTROY THE INTAKE!!!
DO NOT RUN THIS CODE, IT IS UNFINSIHED AND WILL DESTROY THE INTAKE!!!

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

  //Constants (maybe move over to Constants.java later)
  private final double MAX_ANGLE_IN = 0; // Maximum angle of intake based on interior of robot (placeholder)
  private final double MAX_ANGLE_OUT = 0; // Maximum angle of intake based on exterior of robot (placeholder)
  private final double intakeSpeed = 0.5; // Constant pivot speed


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
    if (m_Intake.ThroughboreEncoder.getPosition() < MAX_ANGLE_IN && m_Intake.ThroughboreEncoder.getPosition() > MAX_ANGLE_OUT){
      m_Intake.pivotMotor.set(intakeSpeed);
    } else {
      m_Intake.pivotMotor.set(0);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    // I plan to have this reset the state of the intake to be inside the robot in the future. (maybe)
    m_Intake.pivotMotor.set(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return true; // just in case this code is ran (it isn't supposed to be run)
  }
}