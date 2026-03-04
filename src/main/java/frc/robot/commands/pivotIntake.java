// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
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
    SmartDashboard.putBoolean("Pivot Out", true);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    /*
    if (m_Intake.getPivotEncoderPos() >= SubsystemConstants.INTAKE_MAX_ANGLE_IN && spd<0) {
      m_Intake.setPivotSPD(0); // Motor shouldn't be running once these constants are reached.
    } else if (m_Intake.getPivotEncoderPos() <= SubsystemConstants.INTAKE_MAX_ANGLE_OUT && spd>0){
      m_Intake.setPivotSPD(0);
    } else {
      m_Intake.setPivotSPD(spd);
    }
    */

    m_Intake.pivot(spd);
    System.out.println(m_Intake.getPivotEncoderPos());
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_Intake.setPivotSPD(0);
    SmartDashboard.putBoolean("Pivot Out", false);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return (m_Intake.getPivotEncoderPos() >= SubsystemConstants.INTAKE_MAX_ANGLE_IN && spd<0)||(m_Intake.getPivotEncoderPos() <= SubsystemConstants.INTAKE_MAX_ANGLE_OUT && spd>0);
  }
}