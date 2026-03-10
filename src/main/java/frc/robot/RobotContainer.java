// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
/* 

     @@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@           @@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@      
     @@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@          @@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@      
     @@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@          @@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@      
     @@@@@@@@@@@@@@@           @@@@@@@@@@@@@          @@@@@@@@@@@@@            @@@@@@@@@@@@@@@     
    @@@@@@@@@@@@@@@@            @@@@@@@@@@@@@        @@@@@@@@@@@@@@            @@@@@@@@@@@@@@@     
    @@@@@@@@@@@@@@@@@@@@@@@@    @@@@@@@@@@@@@        @@@@@@@@@@@@@    @@@@@@@@@@@@@@@@@@@@@@@@@    
    @@@@@@@@@@@@@@@@@@@@@@@@    @@@@@@@@@@@@@@      @@@@@@@@@@@@@@    @@@@@@@@@@@@@@@@@@@@@@@@@    
         @@@@@@@@@@@@@@@@@@@@   @@@@@@@@@@@@@@      @@@@@@@@@@@@@@    @@@@@@@@@@@@@@@@@@@          
           @@@@@@@@@@@@@@@      @@@@@@@@@@@@@@@     @@@@@@@@@@@@@@       @@@@@@@@@@@@@@@           
          @@@@@@@@@@@@@@@       @@@@@@@@@@@@@@@     @@@@@@@@@@@@@@       @@@@@@@@@@@@@@@@          
         @@@@@@@@@@@@@@@        @@@@@@@@@@@@@@@    @@@@@@@@@@@@@@@        @@@@@@@@@@@@@@@          
     @@@@@@@@@@@@@@@             @@@@@@@@@@@@@@@   @@@@@@@@@@@@@@@            @@@@@@@@@@@@@@@@     
    @@@@@@@@@@@@@@@@             @@@@@@@@@@@@@@@   @@@@@@@@@@@@@@@             @@@@@@@@@@@@@@@@                                                                                                       
*/
package frc.robot;

import java.io.File;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.OperatorConstants;
import frc.robot.Constants.SubsystemConstants;
import frc.robot.commands.pivotIntake;
import frc.robot.commands.ClimbingCmd;
import frc.robot.commands.ManualClimbCmd;
import frc.robot.commands.ShootCmd;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.Feeder;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.intakeSubsystem;
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

  CommandXboxController m_primary = Constants.OperatorConstants.PRIMARY;
  CommandXboxController m_secondary = Constants.OperatorConstants.SECONDARY;

  // Driver speeds

  private final SwerveSubsystem drivebase = new SwerveSubsystem(new File(Filesystem.getDeployDirectory(),
      "swerve/neo"));
  private final intakeSubsystem m_Intake = new intakeSubsystem();
  private final Climber m_Climber = new Climber();
  private final Feeder m_Feeder = new Feeder();
  private final ShooterSubsystem m_Shooter = new ShooterSubsystem();
  private final ClimbingCmd m_startup = new ClimbingCmd(m_Climber);
  //private final ShootCmd shootCmd;

  public DoubleSupplier getPosTwist = () -> -m_primary.getRightX()*.75;// * ((m_primary.getLeftX() - OperatorConstants.THRUST_SCALAR));
  public DoubleSupplier followTag = () -> {
        if (LimelightHelpers.getTV()) {
          return -Math.max(-0.75, Math.min(LimelightHelpers.getTX() / 27.0, 0.75));
        } else return 0;
      };

  SwerveInputStream driveFollowTag = SwerveInputStream.of(drivebase.getSwerveDrive(), 
  () -> {
    if (!LimelightHelpers.getTV()) return 0;
    double ta = LimelightHelpers.getTA();
    if (ta < 1.7) {
      return (1 / -ta);
    } else if (ta > 4) {
      return ta / 15;
    } else return 0;
    }, 
  () -> 0.0
  ).withControllerRotationAxis(followTag);

  
  SwerveInputStream driveAngularVelocity = SwerveInputStream.of(drivebase.getSwerveDrive(),
      () -> (-m_primary.getLeftY()*.75),
      () -> (-m_primary.getLeftX()*.75))
      .withControllerRotationAxis(getPosTwist)
      .deadband(OperatorConstants.DEADBAND)
      .scaleTranslation(.8)
      .allianceRelativeControl(true);
  SwerveInputStream driveAngularVelocitySlow = SwerveInputStream.of(drivebase.getSwerveDrive(),
      () -> (-m_primary.getLeftY()*.25),
      () -> (-m_primary.getLeftX()*.25))
      .withControllerRotationAxis(getPosTwist)
      .deadband(OperatorConstants.DEADBAND)
      .scaleTranslation(.8)
      .allianceRelativeControl(true);
  SwerveInputStream driveAngularVelocityMedium = SwerveInputStream.of(drivebase.getSwerveDrive(),
      () -> (-m_primary.getLeftY()*.5),
      () -> (-m_primary.getLeftX()*.5))
      .withControllerRotationAxis(getPosTwist)
      .deadband(OperatorConstants.DEADBAND)
      .scaleTranslation(.8)
      .allianceRelativeControl(true);


  /**
   * Clones the angular velocity input stream and converts it to a fieldRelative
   * input stream.
   */

   public ClimbingCmd getClimbingCmd() {
    return m_startup;
   }
  
  public DoubleSupplier getNegTwist = () -> m_primary.getLeftX();
  SwerveInputStream driveDirectAngle = driveAngularVelocity.copy()
      .withControllerHeadingAxis(m_primary::getLeftX, getNegTwist)// checkfunction
      .headingWhile(true);

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    //shootCmd = new ShootCmd(m_shooter);
    //TODO make auto command using new climber cmd
    NamedCommands.registerCommand("ready climber", new ManualClimbCmd(m_Climber, .2, true));
    NamedCommands.registerCommand("climb", new ManualClimbCmd(m_Climber, -.2, true));
    NamedCommands.registerCommand("toggle climb", new ClimbingCmd(m_Climber));
    NamedCommands.registerCommand("flywheels", new ShootCmd(m_Shooter, m_Feeder, SubsystemConstants.AGAINST_HUB_SPEED));
    //NamedCommands.registerCommand("start shooting", new ParallelCommandGroup(new RunHopperCmd(m_hopper)),new FeedCmd(m_feeder)); TODO When robot is finished, uncomment
    //NamedCommands.registerCommand("run flywheels", new ShootCmd(m_shooter));
    configureBindings();
    LimelightTAMatrix.InitializeMatrix();
    ShooterDistanceMatrix.InitializeMatrix();
    DriverStation.silenceJoystickConnectionWarning(true);
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
    Command driveFieldOrientedAnglularVelocity = drivebase.driveFieldOriented(driveAngularVelocity);
    Command driveFieldOrientedAnglularVelocityMedium = drivebase.driveFieldOriented(driveAngularVelocityMedium);
    Command driveFieldOrientedAnglularVelocitySlow = drivebase.driveFieldOriented(driveAngularVelocitySlow);
    
    final ChassisSpeeds DEATH_SPEEDS =  drivebase.getDeath();
    //for others reviewing, the DEATH_SPEEDS variable at line 95 has been tested and is safe for robot use
    //drive team is aware of this
    // create triggers for primary buttons
    // if joystick doesn't have the button you need
    BooleanSupplier deathMode = () -> m_primary.getHID().getRawButton(10);
    Trigger deathModeTrig = new Trigger(deathMode);
    m_primary.povDown().onTrue(drivebase.zeroGyroCmd());


    /*
     * ^^^
     * Devin we need a zero gyro command so we dont freak the navx out.
     * 
     */


    //Button.whileTrue(drivebase.driveCmd(new ChassisSpeeds(.5,0,0)));
    /*m_secondary.y().whileTrue(new ManualClimbCmd(m_Climber, .2,true));
    m_secondary.a().whileTrue(new ManualClimbCmd(m_Climber, -.2,true));*/
    //m_secondary.a().whileTrue(new ManualClimbCmd(m_Climber, -.2, true));
    m_secondary.a().onTrue(new ClimbingCmd(m_Climber));
    m_secondary.y().whileTrue(new ManualClimbCmd(m_Climber, .2, true));
    // Auto Commands

    drivebase.setDefaultCommand(driveFieldOrientedAnglularVelocity);
    m_primary.y().onTrue(driveFieldOrientedAnglularVelocity);
    m_primary.b().onTrue(driveFieldOrientedAnglularVelocityMedium);
    m_primary.a().onTrue(driveFieldOrientedAnglularVelocitySlow);

    //m_primary.button(2).onTrue(drivebase.zeroGyroCmd());

    Trigger Intaketrig=m_primary.axisGreaterThan(2, .25);
    Trigger reving = new Trigger(m_Shooter.ready());

    Intaketrig.whileTrue(new SequentialCommandGroup(new pivotIntake(m_Intake,.4),m_Intake.setIntakeSPD(-0.4)));
    Intaketrig.whileFalse(m_Intake.setIntakeSPD(0)).and(m_primary.rightBumper().whileFalse(m_Intake.setIntakeSPD(0)));
    m_primary.leftBumper().onTrue(new pivotIntake(m_Intake,-.4));
    m_primary.rightBumper().whileTrue(m_Intake.setIntakeSPD(.4));
    m_secondary.rightBumper().and(m_Shooter.ready()).whileTrue(m_Feeder.feed());
    m_secondary.rightBumper().and(m_Shooter.ready()).onFalse(m_Feeder.stopFeeding());
    //m_secondary.rightTrigger(.25).whileTrue(new ShootCmd(m_Shooter));
    /* Shooter stuff:
        m_primary.button(1).onChange(shooter.triggerThing()); will add feeder logic later
    shooter.setDefaultCommand(shooter.Shoot());
    */
   m_secondary.axisGreaterThan(3, .50).whileTrue(m_Shooter.UpdatePids(SubsystemConstants.AGAINST_HUB_SPEED)).whileFalse(m_Shooter.UpdatePids(0));
   //reving.whileTrue(m_Feeder.feed()).onFalse(m_Feeder.stopFeeding()); DO NOT DELETE THIS IS IMPORTANT

    // Primary Driver

    //deathModeTrig.whileTrue(drivebase.driveCmd(DEATH_SPEEDS));


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