package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.Constants;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.util.drivers.LimelightHelpers;
import frc.robot.util.math.MathUtils;
import swervelib.SwerveInputStream;

/**
 * Not recommended for auto but could work.
 */
public class ShootIntoHub extends Command {
    private SwerveSubsystem swerve;
    private Translation2d hubPosition = new Translation2d(0, 1); // TODO: Set this to the actual hub position
    private SwerveInputStream swerveInput;
    PIDController pidController = new PIDController(0.05, 0.05, 0); // 0.3 radians/second 

    
    public ShootIntoHub(SwerveSubsystem swerve, SwerveInputStream swerveInput) {
        this.swerve = swerve;
        this.swerveInput = swerveInput;
        addRequirements(swerve);
    }

    /**
     * Gets the target rotation for the robot to face in order to shoot into the hub.
     */
    DoubleSupplier getTargetHeading = () -> {
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
        if (LimelightHelpers.getTV()) {
          return -Math.max(-0.9, Math.min(((LimelightHelpers.getTX() + 26) / 27.0), 0.9));
        } else return 0;
    };

    // 90 - (sin^-1 (gravity * ((size + distance)) / ((rpm * dπ) / 60)^2) / 2

    /*
    where:

    gravity is either 9.8 m/s² or 32 ft/s²
    size is the length of the robot + half the length of the hub (4)
    distance is the distance from the hub's edge
    rpm is the rotations per minute
    diameter is the diameter of the flywheel (either 5/12 feet or .127 meters)
    */

    @Override
    public void initialize() {
    }

    /**
     * This will not feed because im too lazy to do that rn.
     */
    @Override
    public void execute() {
        swerve.estimatePoseWithLimelight(); 

        ChassisSpeeds speeds = new ChassisSpeeds(swerveInput.get().vxMetersPerSecond, swerveInput.get().vyMetersPerSecond, getTargetHeading.getAsDouble() == 0 ? swerveInput.get().omegaRadiansPerSecond : getTargetHeading.getAsDouble());
        
        /*double targetSpeed = 90 - 
        (Math.asin(-9.81 * (((Constants.CHASSIS.position.getX() + 21.5) + swerve.getPose().getTranslation().getDistance(hubPosition))
         / Math.pow((shooter.encoder.getVelocity())
          * (Constants.SHOOTER_WHEEL_DIAMETER * Math.PI), 2))) / 2)
          * (180 / Math.PI);*/

         // Its possible the target heading will be "behind" but I doubt that will be an issue.
        swerve.getSwerveDrive().driveFieldOriented(speeds);
    }

    @Override
    public void end(boolean interrupted) {
        //CommandScheduler.getInstance().schedule(shooter.StopShooting());
    }
}
