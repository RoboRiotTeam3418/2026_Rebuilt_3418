package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.util.drivers.LimelightHelpers;
import swervelib.SwerveInputStream;

/**
 * Not recommended for auto but could work.
 */
public class ShootIntoHub extends Command {
    private ShooterSubsystem shooter;
    private SwerveSubsystem swerve;
    private Translation2d hubPosition = new Translation2d(0, 0); // TODO: Set this to the actual hub position
    private SwerveInputStream swerveInput;

    public ShootIntoHub(ShooterSubsystem shooter, SwerveSubsystem swerve, SwerveInputStream swerveInput) {
        this.shooter = shooter;
        this.swerve = swerve;
        this.swerveInput = swerveInput;
        addRequirements(shooter, swerve);
    }

    /**
     * Gets the target rotation for the robot to face in order to shoot into the hub.
     */
    DoubleSupplier getTargetHeading = () -> {
        Translation2d robotLocation = swerve.getPose().getTranslation();
        Translation2d delta = hubPosition.minus(robotLocation);
        double targetAngle = -Math.atan2(delta.getX(), delta.getY());
        return targetAngle;
    };

    // 90 - (sin⁻¹(gravity * ((size + distance)) / ((rpm * dπ) / 60)²) / 2

    /*
    where:

    gravity is either 9.8 m/s² or 32 ft/s²
    size is the length of the robot + half the length of the hub (4)
    distance is the distance from the hub's edge
    rpm is the rotations per minute
    diameter is the diameter of the flywheel (either 5/12 feet or .127 meters)
    */

    /**
     * This will not feed because im too lazy to do that rn.
     */
    @Override
    public void execute() {
        swerve.estimatePoseWithLimelight();
        
        /*double targetSpeed = 90 - 
        (Math.asin(-9.81 * (((Constants.CHASSIS.position.getX() + 21.5) + swerve.getPose().getTranslation().getDistance(hubPosition))
         / Math.pow((shooter.encoder.getVelocity())
          * (Constants.SHOOTER_WHEEL_DIAMETER * Math.PI), 2))) / 2)
          * (180 / Math.PI);*/

        //shooter.setTargetSpeed(targetSpeed);

        swerveInput.withControllerRotationAxis(getTargetHeading);
        swerve.getSwerveDrive().driveFieldOriented(swerveInput.get());
    }

    @Override
    public void end(boolean interrupted) {
        CommandScheduler.getInstance().schedule(shooter.StopShooting());
    }
}
