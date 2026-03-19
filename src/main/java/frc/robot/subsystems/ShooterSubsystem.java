package frc.robot.subsystems;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.AprilTagConstants;
import frc.robot.Constants.SubsystemConstants;
import frc.robot.util.LimelightTAMatrix;
import frc.robot.util.ShooterDistanceMatrix;
import frc.robot.util.drivers.LimelightHelpers;

/** Shooter subsystem for controlling the flywheel(s) */

public class ShooterSubsystem extends SubsystemBase {
    public static ShooterSubsystem Instance;
    private static double 
    p = 0.0005,
    i = 0.000001,
    d = 0.00;
    
    /**
     * This is the PID controller for the shooter flywheels. DO NOT CHANGE ANYTHING INSIDE, READ ONLY!
     */
    public SparkClosedLoopController pidController;
    /**
     * If true, override drive control with april tag position
     */
    public boolean overrideDrive = false;
    public boolean readyToShoot = false;
    double setpoint = 0;
    SparkMax sparkMaxA, sparkMaxB;
    public RelativeEncoder encoder;

    /**
     * Constructor for shooter subsystem, initializes motors, encoders, and PID controller. Also sets limelight pipeline. Logs PID values to smart dashboard in test mode.
     */
    public ShooterSubsystem() {
        Instance = this;

        Log("Shooter subsystem loading...\nTest mode is enabled, do not use this in comp it sends a LOT to smart dashboard!!\nP: " + p + ", I: " + i + ", D: " + d);

        sparkMaxA = new SparkMax(SubsystemConstants.SHOOTER_MOTOR_A, SparkMax.MotorType.kBrushless);
        sparkMaxB = new SparkMax(SubsystemConstants.SHOOTER_MOTOR_B, SparkMax.MotorType.kBrushless);

        encoder = sparkMaxA.getEncoder();

        SparkMaxConfig config = new SparkMaxConfig();
        config.closedLoop.pid(p, i, d).outputRange(0, 5000);
        config.inverted(true);

        SparkMaxConfig followerConfig = new SparkMaxConfig();
        followerConfig.follow(sparkMaxA);

        pidController = sparkMaxA.getClosedLoopController();
        sparkMaxA.configure(config, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
        sparkMaxB.configure(followerConfig, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);

        LimelightHelpers.setPipelineIndex("limelight", Constants.LIMELIGHT_PIPELINE_ID);
    }

    /**
     * @return True if the hub is in the limelights sight.
     */
    public boolean hubInSight() {
        if (!LimelightHelpers.getTV() || Constants.SAD_LIMELIGHT_MODE) return false;

        return LimelightHelpers.getTID() == AprilTagConstants.HUB_CENTER_BLUE || LimelightHelpers.getTID() == AprilTagConstants.HUB_CENTER_RED;
    }

    /**
     * April tag position at hub (if seen)
     * @return limelight horizontal offset to april tag at hub
     */
    public DoubleSupplier aprilTagPos = () -> {
        if (!LimelightHelpers.getTV() || Constants.SAD_LIMELIGHT_MODE) return 0;

        if (hubInSight()) {
            return LimelightHelpers.getTX();
        }

        return 0;
    };

    /**
     * Calculates flywheel speed based on limelight data. If no target, returns 0.7
     * @see LimelightTAMatrix.java
     * @see ShooterDistanceMatrix.java
     * @return flywheel speed (0.05 to 1)
     */
    public double limelightCalculator() {
        if (!LimelightHelpers.getTV() || Constants.SAD_LIMELIGHT_MODE) return 0.36; // set flywheel speed regardless of vision

        if (hubInSight()) {
            double ta = LimelightHelpers.getTA();
            double speed = ShooterDistanceMatrix.get(LimelightTAMatrix.get(ta)); // Ta matrix gets the distance from ta, then shooter distance converts that to flywheel speed
            if (DriverStation.isTestEnabled()) {
                SmartDashboard.putNumber("Distance from limelight ", speed);
            }
            return speed;
        }


        return 0.7;
    }
    /**
     * Toggle override for drive control
     */
    public Command ToggleOverride() {
        return runOnce(() -> {
            overrideDrive = !overrideDrive;
        });
    }

    double targetSpeed = 5000; // This is in rpm!! Tune on thursday!

    public final double THRESHOLD = 100; 


    public Command UpdatePids(double speed) {
        return run(() -> {
            pidController.setSetpoint(speed, ControlType.kVelocity);
            readyToShoot = (Math.abs(encoder.getVelocity() - speed) <= THRESHOLD) && speed > 0;
        });
    }

    /**
     * Command to shoot balls
     */
    public Command setSetpoint(double value) {
        return runOnce(() -> {
            //targetSpeed = limelightCalculator();
            //pidController.setSetpoint(targetSpeed, ControlType.kVelocity);
            //beforeClamp = pidController.getMAXMotionSetpointPosition();
            

            //setSpeeds(-speed);

            setpoint = value;

            if (DriverStation.isTestEnabled()) {
                SmartDashboard.putNumber("Shooter PID target", targetSpeed);
                SmartDashboard.putBoolean("Should start feeding", readyToShoot);
                SmartDashboard.putNumber("Threshold", THRESHOLD);
                SmartDashboard.putNumber("Math", Math.abs((encoder.getVelocity() / 6784) - targetSpeed));
            }
        });
    }

    public Command StopShooting() {
        return runOnce(() -> {
            setSpeeds(0);
            readyToShoot = false;
            pidController.setSetpoint(0, ControlType.kVelocity);
            targetSpeed = 0;

            if (DriverStation.isTestEnabled()) {
                SmartDashboard.putNumber("Shooter PID target", targetSpeed);
            }
        });
    }

    /**
     * Sets the speed of both motors
     * @param speed the target speed
     */
    public void setSpeeds(double speed) {
        sparkMaxA.set(speed);
        sparkMaxB.set(speed);
        if (DriverStation.isTestEnabled())
            SmartDashboard.putNumber("current speed", speed);
    }
    public double getSpeeds() {
        return sparkMaxA.getEncoder().getVelocity();
    }

    public void stopMotors() {
        sparkMaxA.stopMotor();
        sparkMaxB.stopMotor();
    }

    /**
     * Debug command to update PID values
     * @param kP P
     * @param kI I
     * @param kD D
     */
    public Command UpdatePID(double kP, double kI, double kD) {
        return runOnce(() -> {
            Log("Pids updated to: " + kP + ", " + kI + ", " + kD);

           // pidController.setPID(kP, kI, kD);
            p = kP;
            i = kI;
            d = kD;
        });
    }

    /**
     * Debug command to update PID values
     */
    public Command UpdatePID() {
        return runOnce(() -> {
            //pidController.setPID(p, i, d);
        });
    }
    @Override
    public void periodic() {
        SmartDashboard.putBoolean("Ready to Shoot?",ready().getAsBoolean());
        SmartDashboard.putNumber("current speed", encoder.getVelocity());
    }

    /**
     * Log to console only in test mode
     * @param objects objects to log
     */
    public void Log(Object objects) {
        if (DriverStation.isTestEnabled()) {
            System.out.println(objects);
        }
    }
    public BooleanSupplier ready() {
        return () -> readyToShoot;
    }

    public boolean shoudFeed(double speed) {
        return (Math.abs(encoder.getVelocity() - speed) <= THRESHOLD) && speed > 0;
    }

    /**
     * Test command to verify subsystem is working, recommended use is for autonomous command testing
     */
    public Command test() {
        return runOnce(() -> {
            System.out.println("Hi");
        });
    }
}
