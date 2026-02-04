// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.io.File;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.ClimbCmd;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.util.LimelightTAMatrix;
import frc.robot.util.ShooterDistanceMatrix;
import frc.robot.util.drivers.LimelightHelpers;
import swervelib.SwerveInputStream;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in
 * the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of
 * the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {

  CommandJoystick m_primary = Constants.OperatorConstants.PRIMARY;
  CommandXboxController m_secondary = Constants.OperatorConstants.SECONDARY;

  // Driver speeds

  private final SwerveSubsystem drivebase = new SwerveSubsystem(new File(Filesystem.getDeployDirectory(),
      "swerve/neo"));
  private final ShooterSubsystem m_shooter = new ShooterSubsystem();



  // More shooter stuff: private final ShooterSubsystem shooter = new ShooterSubsystem();

  public DoubleSupplier getPosTwist = () -> m_primary.getRawAxis(5) * ((m_primary.getZ() - (23.0 / 9.0)) / (40.0 / 9.0));
  public DoubleSupplier followTag = () -> {
        if (LimelightHelpers.getTV("limelight")) {
          return -Math.max(-0.75, Math.min(LimelightHelpers.getTX("limelight") / 27.0, 0.75));
        } else return 0;
      };

  SwerveInputStream driveFollowTag = SwerveInputStream.of(drivebase.getSwerveDrive(), 
  () -> {
    if (!LimelightHelpers.getTV("limelight")) return 0;
    double ta = LimelightHelpers.getTA("limelight");
    if (ta < 1.7) {
      return (1 / -ta);
    } else if (ta > 4) {
      return ta / 15;
    } else return 0;
    }, 
  () -> 0.0
  ).withControllerRotationAxis(followTag);

  
  SwerveInputStream driveAngularVelocity = SwerveInputStream.of(drivebase.getSwerveDrive(),
      () -> (m_primary.getY() * -((m_primary.getZ() - (23.0 / 9.0)) / (40.0 / 9.0))) / 2,
      () -> (m_primary.getX() * ((m_primary.getZ() - (23.0 / 9.0)) / (40.0 / 9.0))) / 2)
      .withControllerRotationAxis(getPosTwist)
      .deadband(OperatorConstants.DEADBAND)
      .allianceRelativeControl(true);


  /**
   * Clones the angular velocity input stream and converts it to a fieldRelative
   * input stream.
   */
  
  public DoubleSupplier getNegTwist = () -> m_primary.getTwist();
  SwerveInputStream driveDirectAngle = driveAngularVelocity.copy()
      .withControllerHeadingAxis(m_primary::getTwist, getNegTwist)// checkfunction
      .headingWhile(true);

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    configureBindings();
    LimelightTAMatrix.InitializeMatrix();
    ShooterDistanceMatrix.InitializeMatrix();
    DriverStation.silenceJoystickConnectionWarning(true);
    NamedCommands.registerCommand("test", m_shooter.test());
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be
   * created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with
   * an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for
   * {@link
   * CommandXboxController
   * Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or
   * {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */

  

  private void configureBindings() {
    // DRIVETRAIN COMMAND ASSIGNMENTS R
    Command driveFieldOrientedAnglularVelocity = drivebase.driveFieldOriented(driveAngularVelocity);//drivebase.driveFieldOriented(driveAngularVelocity);
    final ChassisSpeeds DEATH_SPEEDS =  drivebase.getDeath();
    //for others reviewing, the DEATH_SPEEDS variable at line 95 has been tested and is safe for robot use
    //drive team is aware of this
    // create triggers for primary buttons
    // if joystick doesn't have the button you need
    BooleanSupplier zeroGyro = () -> m_primary.getHID().getRawButton(2);
    Trigger zeroGyroTrig = new Trigger(zeroGyro);
    BooleanSupplier deathMode = () -> m_primary.getHID().getRawButton(10);
    Trigger deathModeTrig = new Trigger(deathMode);
    BooleanSupplier button = () -> m_primary.getHID().getRawButton(3);
    Trigger trig = new Trigger(button);

    // Auto Orient (I dont believe we need this - Darwin )

    // Auto Commands

    drivebase.setDefaultCommand(driveFieldOrientedAnglularVelocity);
    zeroGyroTrig.whileTrue(drivebase.driveCmd(new ChassisSpeeds(.1,0,0)));
    trig.whileTrue(drivebase.driveCmd(new ChassisSpeeds(0,.1,0)));
    /* Shooter stuff:
    m_primary.button(1).onChange(shooter.triggerThing());
    shooter.setDefaultCommand(shooter.Shoot());
    */

    // Primary Driver

    deathModeTrig.whileTrue(drivebase.driveCmd(DEATH_SPEEDS));
    // fullStopTrig.whileTrue(Commands.runOnce(drivebase::lock,
    // drivebase).repeatedly());
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    return drivebase.getAutonomousCommand("New Auto");
  }

  public void setMotorBrake(boolean brake) {
    drivebase.setMotorBrake(brake);
  }
}