// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
/* 

      @@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@            @@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@      
      @@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@            @@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@      
     @@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@            @@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@      
     @@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@           @@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@      
     @@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@           @@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@      
     @@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@          @@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@      
     @@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@          @@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@      
     @@@@@@@@@@@@@@@           @@@@@@@@@@@@@          @@@@@@@@@@@@@            @@@@@@@@@@@@@@@     
     @@@@@@@@@@@@@@@            @@@@@@@@@@@@          @@@@@@@@@@@@@            @@@@@@@@@@@@@@@     
    @@@@@@@@@@@@@@@@            @@@@@@@@@@@@@         @@@@@@@@@@@@@            @@@@@@@@@@@@@@@     
    @@@@@@@@@@@@@@@@            @@@@@@@@@@@@@         @@@@@@@@@@@@@            @@@@@@@@@@@@@@@     
    @@@@@@@@@@@@@@@@            @@@@@@@@@@@@@        @@@@@@@@@@@@@@            @@@@@@@@@@@@@@@     
    @@@@@@@@@@@@@@@@            @@@@@@@@@@@@@        @@@@@@@@@@@@@@            @@@@@@@@@@@@@@@     
    @@@@@@@@@@@@@@@@            @@@@@@@@@@@@@        @@@@@@@@@@@@@@            @@@@@@@@@@@@@@@     
    @@@@@@@@@@@@@@@@@@@@@@@@    @@@@@@@@@@@@@        @@@@@@@@@@@@@    @@@@@@@@@@@@@@@@@@@@@@@@@    
    @@@@@@@@@@@@@@@@@@@@@@@@    @@@@@@@@@@@@@@       @@@@@@@@@@@@@    @@@@@@@@@@@@@@@@@@@@@@@@@    
   @@@@@@@@@@@@@@@@@@@@@@@@@    @@@@@@@@@@@@@@       @@@@@@@@@@@@@    @@@@@@@@@@@@@@@@@@@@@@@@@    
   @@@@@@@@@@@@@@@@@@@@@@@@@    @@@@@@@@@@@@@@       @@@@@@@@@@@@@    @@@@@@@@@@@@@@@@@@@@@@@@@    
    @@@@@@@@@@@@@@@@@@@@@@@@    @@@@@@@@@@@@@@      @@@@@@@@@@@@@@    @@@@@@@@@@@@@@@@@@@@@@@@@    
         @@@@@@@@@@@@@@@@@@@@   @@@@@@@@@@@@@@      @@@@@@@@@@@@@@    @@@@@@@@@@@@@@@@@@@          
             @@@@@@@@@@@@@@@    @@@@@@@@@@@@@@      @@@@@@@@@@@@@@    @@@@@@@@@@@@@@@              
            @@@@@@@@@@@@@@@     @@@@@@@@@@@@@@      @@@@@@@@@@@@@@     @@@@@@@@@@@@@@@             
           @@@@@@@@@@@@@@@@     @@@@@@@@@@@@@@@     @@@@@@@@@@@@@@      @@@@@@@@@@@@@@@            
           @@@@@@@@@@@@@@@      @@@@@@@@@@@@@@@     @@@@@@@@@@@@@@       @@@@@@@@@@@@@@@           
          @@@@@@@@@@@@@@@       @@@@@@@@@@@@@@@     @@@@@@@@@@@@@@       @@@@@@@@@@@@@@@@          
         @@@@@@@@@@@@@@@        @@@@@@@@@@@@@@@    @@@@@@@@@@@@@@@        @@@@@@@@@@@@@@@          
        @@@@@@@@@@@@@@@@         @@@@@@@@@@@@@@    @@@@@@@@@@@@@@@         @@@@@@@@@@@@@@@         
       @@@@@@@@@@@@@@@@          @@@@@@@@@@@@@@    @@@@@@@@@@@@@@@          @@@@@@@@@@@@@@@        
      @@@@@@@@@@@@@@@@           @@@@@@@@@@@@@@    @@@@@@@@@@@@@@@          @@@@@@@@@@@@@@@@       
     @@@@@@@@@@@@@@@@            @@@@@@@@@@@@@@    @@@@@@@@@@@@@@@           @@@@@@@@@@@@@@@@      
     @@@@@@@@@@@@@@@             @@@@@@@@@@@@@@@   @@@@@@@@@@@@@@@            @@@@@@@@@@@@@@@@     
    @@@@@@@@@@@@@@@@             @@@@@@@@@@@@@@@   @@@@@@@@@@@@@@@             @@@@@@@@@@@@@@@@    
   @@@@@@@@@@@@@@@@              @@@@@@@@@@@@@@@  @@@@@@@@@@@@@@@              @@@@@@@@@@@@@@@@    
  @@@@@@@@@@@@@@@@               @@@@@@@@@@@@@@@  @@@@@@@@@@@@@@@               @@@@@@@@@@@@@@@@   
 @@@@@@@@@@@@@@@@                @@@@@@@@@@@@@@@  @@@@@@@@@@@@@@@                @@@@@@@@@@@@@@@@  
                                                                                                   
*/
package frc.robot;

import java.io.File;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import java.util.jar.Attributes.Name;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.DoubleTopic;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.OperatorConstants;
import frc.robot.Constants.SubsystemConstants;
import frc.robot.commands.pivotIntake;
import frc.robot.commands.ShootCmd;
import frc.robot.commands.ShootIntoHub;
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
  private final Feeder m_Feeder = new Feeder();
  private final ShooterSubsystem m_Shooter = new ShooterSubsystem();

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

    //AUTO CHOOSER
  NetworkTableInstance inst = NetworkTableInstance.getDefault();
  private final SendableChooser<Double> side = new SendableChooser<>();
  DoubleTopic chosenSide = inst.getDoubleTopic("/SmartDashboard/Left or Right");
  DoublePublisher sidePublisher = chosenSide.publish();
  private final SendableChooser<Double> neutral = new SendableChooser<>();
  DoubleTopic chosenNeutral = inst.getDoubleTopic("/SmartDashboard/Grab or Sabotage");
  DoublePublisher neutralPublisher = chosenNeutral.publish();
  private final SendableChooser<Double> exit = new SendableChooser<>();
  DoubleTopic chosenExit = inst.getDoubleTopic("/SmartDashboard/Exit");
  DoublePublisher exitPublisher = chosenExit.publish();

  
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

  
  public DoubleSupplier getNegTwist = () -> m_primary.getLeftX();
  SwerveInputStream driveDirectAngle = driveAngularVelocity.copy()
      .withControllerHeadingAxis(m_primary::getLeftX, getNegTwist)// checkfunction
      .headingWhile(true);

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    NamedCommands.registerCommand("extend intake", new pivotIntake(m_Intake, -.4));
    NamedCommands.registerCommand("shoot short", new ShootCmd(m_Shooter, m_Feeder, 4300));
    NamedCommands.registerCommand("shoot long", new ShootCmd(m_Shooter, m_Feeder,5000));
    NamedCommands.registerCommand("intake", m_Intake.setIntakeSPD(-.45));
    NamedCommands.registerCommand("stop intaking", m_Intake.setIntakeSPD(0));
    configureBindings();
    LimelightTAMatrix.InitializeMatrix();
    ShooterDistanceMatrix.InitializeMatrix();
    DriverStation.silenceJoystickConnectionWarning(true);

    //autonomous stuff
    side.setDefaultOption("Left", 1.0);
    neutral.setDefaultOption("Grab", 10.0);
    exit.setDefaultOption("Same", 1.0);
    side.addOption("Right", -1.0);
    neutral.addOption("Sabotage", 20.0);
    exit.addOption("Opposite", 2.0);
    exit.addOption("Bulldoze", 3.0);
    SmartDashboard.putData("Left or Right", side);
    SmartDashboard.putData("Grab or Sabotage", neutral);
    SmartDashboard.putData("Exit", exit);
    inst.startClient4("Autonomous");
    inst.setServerTeam(3418);
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

    m_primary.axisGreaterThan(3, .50).whileTrue(new ParallelCommandGroup(new ShootIntoHub(drivebase, driveAngularVelocity)/* , m_Shooter.UpdatePids(0)*/));//.whileFalse(m_Shooter.StopShooting());

    // Swerve Subsystem
    Command driveFieldOrientedAnglularVelocity = drivebase.driveFieldOriented(driveAngularVelocity);
    Command driveFieldOrientedAnglularVelocityMedium = drivebase.driveFieldOriented(driveAngularVelocityMedium);
    Command driveFieldOrientedAnglularVelocitySlow = drivebase.driveFieldOriented(driveAngularVelocitySlow);
    final ChassisSpeeds DEATH_SPEEDS =  drivebase.getDeath();
    
    drivebase.setDefaultCommand(driveFieldOrientedAnglularVelocity);

    m_primary.y().onTrue(driveFieldOrientedAnglularVelocity);
    m_primary.b().onTrue(driveFieldOrientedAnglularVelocityMedium);
    m_primary.a().onTrue(driveFieldOrientedAnglularVelocitySlow);
    m_primary.povDown().onTrue(drivebase.zeroGyroCmd());

    // Intake Subsystem

    Trigger Intaketrig=m_primary.axisGreaterThan(2, .25);
    Intaketrig.whileTrue(new SequentialCommandGroup(new pivotIntake(m_Intake,-.4),m_Intake.setIntakeSPD(-0.45)));
    Intaketrig.whileFalse(m_Intake.setIntakeSPD(0)).and(m_primary.rightBumper().whileFalse(m_Intake.setIntakeSPD(0)));
    m_primary.leftBumper().onTrue(new pivotIntake(m_Intake,.2));
    //m_primary.rightBumper().whileTrue(m_Intake.setIntakeSPD(.4));

    // Shooter + Feeder Subsystems
    
    //m_secondary.rightBumper().and(m_Shooter.ready()).whileTrue(m_Feeder.feed());
    //m_secondary.rightBumper().and(m_Shooter.ready()).onFalse(m_Feeder.stopFeeding());
    //m_Shooter.setDefaultCommand(m_Shooter.TickSpeed());
    //m_secondary.rightTrigger(.25) TODO: Add safeguard cause it's not working.
    m_secondary.a().and(m_Shooter.ready()).whileTrue(m_Feeder.feed()).onFalse(m_Feeder.stopFeeding());// DO NOT DELETE THIS IS IMPORTANT

   // Hopper Subsystem
  }
  //this is the equation for autonomous IDs
  public double selectedAuto() {
    double autoNum=side.getSelected()*(neutral.getSelected()+exit.getSelected());
    return autoNum;
  }
  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    String chosenAuto="";
    //I LOVE SWITCH STATEMENTS
    switch ((int)selectedAuto()) {
      case 11:
        chosenAuto=("Grab Right, don't grab left, exit right");
        break;
      case 12:
        chosenAuto=("Grab Right, don't grab left, exit left");
        break;
      case 13:
        chosenAuto=("Grab Right, grab left, exit left");
        break;
      case 21:
        chosenAuto=("Sabotage Right, don't grab left, exit right");
        break;
      case 22:
        chosenAuto=("Sabotage Right, don't grab left, exit left");
        break;
      case 23:
        chosenAuto=("Sabotage Right, grab left, exit left");
        break;
      case -11:
        chosenAuto=("Grab Left, don't grab right, exit left");
        break;
      case -12:
        chosenAuto=("Grab Left, don't grab right, exit right");
        break;
      case -13:
        chosenAuto=("Grab Left, grab right, exit right");
        break;
      case -21:
        chosenAuto=("Sabotage Left, don't grab right, exit left");
        break;
      case -22:
        chosenAuto=("Sabotage Left, don't grab right, exit right");
        break;
      case -23:
        chosenAuto=("Sabotage Left, grab right, exit right");
        break;
    }
    System.out.println(chosenAuto);
    return drivebase.getAutonomousCommand(chosenAuto);
  }

  public void setMotorBrake(boolean brake) {
    drivebase.setMotorBrake(brake);
  }
}