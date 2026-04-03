package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.subsystems.Feeder;
import frc.robot.subsystems.Servos;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.util.ShooterDistanceMatrix;
import swervelib.SwerveInputStream;

/**
 * Not recommended for auto but could work.
 */
public class ShootIntoHub extends Command {
    private SwerveSubsystem swerve;
    private Servos servos;
    private Translation2d hubPosition = new Translation2d(4.5, 4); // TODO: Set this to the actual hub position
    private SwerveInputStream swerveInput;
    PIDController pidController = new PIDController(3.1, 0.8, 0.7);
    Field2d field = new Field2d();

    
    public ShootIntoHub(SwerveSubsystem swerve, SwerveInputStream swerveInput, Servos servos) {
        this.swerve = swerve;
        this.servos = servos;
        this.swerveInput = swerveInput;
        addRequirements(swerve, servos);
    }

    double servoAngle() {
        double distanceToHub = swerve.getPose().getTranslation().getDistance(hubPosition);

        double angle = ShooterDistanceMatrix.get(distanceToHub) - 0.3;
        if (DriverStation.isTest()) {
            SmartDashboard.putNumber("Distance to hub", distanceToHub);
            SmartDashboard.putNumber("Servo angle", angle);
        }

        return angle;
    }

    /**
     * Gets the target rotation for the robot to face in order to shoot into the hub.
     */
    DoubleSupplier getTargetHeading = () -> {
        Pose2d currentPose = swerve.getPose();
        Translation2d robotTranslation = currentPose.getTranslation();

        Rotation2d angleToHub = hubPosition.minus(robotTranslation).getAngle();
        double rotationCommand = currentPose.getRotation().minus(angleToHub).getRadians();

        if (DriverStation.isTest()) {
            SmartDashboard.putNumber("Angle to hub", angleToHub.getRadians());
            SmartDashboard.putString("Robot pose", currentPose.toString());
            SmartDashboard.putNumber("Rotation command", rotationCommand);
        }

        return pidController.calculate(rotationCommand);


        /*Translation2d robotLocation = swerve.getPose().getTranslation();
        //SmartDashboard.putData(field);
        ChassisSpeeds speeds = swerve.getSwerveDrive().getRobotVelocity();
        // I found while testing the hub position is actually relative to the robot so this *should* correct for that.
        hubPosition = hubPosition.minus(new Translation2d(speeds.vxMetersPerSecond, speeds.vyMetersPerSecond)); // Gracias father ford
        Translation2d delta = hubPosition.minus(robotLocation);
        double targetAngle = -Math.atan2(delta.getY(), delta.getX());

        SmartDashboard.putString("Virtual Robot position", "X: " + robotLocation.getX() + ", Y: " + robotLocation.getY());
        SmartDashboard.putString("Virtual hub position", "X: " + hubPosition.getX() + ", Y: " + hubPosition.getY());
        SmartDashboard.putNumber("Target Angle", targetAngle);
        SmartDashboard.putNumber("target heading", pidController.calculate(targetAngle - swerve.getPose().getRotation().getRadians()));

        if (targetAngle < 0) {
            return pidController.calculate(swerve.getPose().getRotation().getRadians() - targetAngle);
            //return pidController.calculate(targetAngle - swerve.getPose().getRotation().getRadians());
        } else {
            return pidController.calculate(targetAngle - swerve.getPose().getRotation().getRadians());
            //return pidController.calculate(swerve.getPose().getRotation().getRadians() - targetAngle);
        }
        
        //return pidController.calculate(targetAngle - swerve.getPose().getRotation().getRadians());
        
*/
        //if (LimelightHelpers.getTV()) {
        //  return -Math.max(-0.9, Math.min(((LimelightHelpers.getTX() + 26) / 27.0), 0.9));
        //} else return 0;
    };

    // 90 - (sin^-1 (gravity * ((size + distance)) / ((rpm * dπ) / 60)^2) / 2

    /*
    where:

    gravity is either 9.8 m/s² or 32 ft/s²
    size is the length of the robot + half the length of the hub (4)
    distance is the distance from the hub's edge
    rpm is the rotations per minute
    diameter is the diameter of the flywheel (either 5/12 feet or .127 mete]rs)
    */

    @Override
    public void initialize() {
        swerve.getSwerveDrive().setHeadingCorrection(true);

        pidController.enableContinuousInput(-Math.PI, Math.PI);
        pidController.reset();
        pidController.setTolerance(Math.toRadians(5.0));

        hubPosition = DriverStation.getAlliance().orElse(Alliance.Red) == Alliance.Red ? new Translation2d(4, 4.5) : new Translation2d(15.75, 4.5); // TODO: Update with proper positions
    }

    /**
     * This will not feed because im too lazy to do that rn.
     */
    @Override
    public void execute() {
        swerve.estimatePoseWithLimelight(true);

        double rotation = getTargetHeading.getAsDouble();
        ChassisSpeeds speeds = new ChassisSpeeds(swerveInput.get().vxMetersPerSecond, swerveInput.get().vyMetersPerSecond, rotation);

        servos.setAngle(servoAngle());
        //shooter.setTargetSpeed(3500);

        if (DriverStation.isTest())
            SmartDashboard.putNumber("Rot PID Out", rotation);

        swerve.getSwerveDrive().driveFieldOriented(speeds);
    }

    @Override
    public void end(boolean interrupted) {
        swerve.getSwerveDrive().setHeadingCorrection(false);
    }
}
