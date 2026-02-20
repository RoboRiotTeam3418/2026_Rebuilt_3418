package frc.robot.subsystems;

import java.util.function.DoubleSupplier;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.math.controller.PIDController;
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
import frc.robot.util.math.MathUtils;

/** Shooter subsystem for controlling the flywheel(s) */

public class ShooterSubsystem extends SubsystemBase {
    public static ShooterSubsystem Instance;

    public double 
    p = 0.1,
    i = 0.01,
    d = 0;
    
    /**
     * This is the PID controller for the shooter flywheels. DO NOT CHANGE ANYTHING INSIDE, READ ONLY!
     */
    public PIDController pidController;
    /**
     * If true, override drive control with april tag position
     */
    public boolean overrideDrive = false;
    static boolean trigger = false;

    SparkMax sparkMaxA, sparkMaxB;
    public AbsoluteEncoder encoderA, encoderB;

    /**
     * Constructor for shooter subsystem, initializes motors, encoders, and PID controller. Also sets limelight pipeline. Logs PID values to smart dashboard in test mode.
     */
    public ShooterSubsystem() {
        Instance = this;

        Log("Shooter subsystem loading...\nTest mode is enabled, do not use this in comp it sends a LOT to smart dashboard!!\nP: " + p + ", I: " + i + ", D: " + d);


        sparkMaxA = new SparkMax(SubsystemConstants.SHOOTER_MOTOR_A, SparkMax.MotorType.kBrushless);
        sparkMaxB = new SparkMax(SubsystemConstants.SHOOTER_MOTOR_B, SparkMax.MotorType.kBrushless);

        encoderA = sparkMaxA.getAbsoluteEncoder();
        encoderB = sparkMaxB.getAbsoluteEncoder();

        pidController = new PIDController(p, i, d);
        pidController.setSetpoint(0);

        LimelightHelpers.setPipelineIndex("limelight", Constants.LIMELIGHT_PIPELINE_ID);
    }

    public boolean hubInSight() {
        if (!LimelightHelpers.getTV("limelight") || Constants.SAD_LIMELIGHT_MODE) return false;

        return LimelightHelpers.getTID() == AprilTagConstants.HUB_CENTER_BLUE || LimelightHelpers.getTID() == AprilTagConstants.HUB_CENTER_RED;
    }

    /**
     * April tag position at hub (if seen)
     * @return limelight horizontal offset to april tag at hub
     */
    public DoubleSupplier aprilTagPos = () -> {
        if (!LimelightHelpers.getTV("limelight") || Constants.SAD_LIMELIGHT_MODE) return 0;

        if (hubInSight()) {
            return LimelightHelpers.getTX("limelight");
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
        if (!LimelightHelpers.getTV("limelight") || Constants.SAD_LIMELIGHT_MODE) return 0.7; // set flywheel speed regardless of vision

        if (hubInSight()) {
            double ta = LimelightHelpers.getTA("limelight");
            double distanceInCm = ShooterDistanceMatrix.get(LimelightTAMatrix.get(ta));
            if (DriverStation.isTestEnabled()) {
                SmartDashboard.putNumber("Distance from limelight ", distanceInCm);
            }
            return distanceInCm;
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

    public Command triggerThing() {return runOnce(() -> {trigger = !trigger; } ); }

    DoubleSupplier getSetpoint = () -> {
        return trigger ? limelightCalculator() : -0.2;
    };

    /**
     * Command to shoot balls
     */
    public Command Shoot() {
        return run(() -> {
            double beforeClamp = pidController.calculate(encoderA.getVelocity(), getSetpoint.getAsDouble()) * 10; // This has been tested and is safe for robot use
            double speed = MathUtils.clamp( beforeClamp, 0, 0.7);

            if (DriverStation.isTestEnabled()) {
                SmartDashboard.putNumber("Shooter PID output before clamp", beforeClamp);
                SmartDashboard.putNumber("Shooter PID output after clamp", speed);
            }

            setSpeeds(speed);
        });
    }

    /**
     * Sets the speed of both motors
     * @param speed the target speed
     */
    public void setSpeeds(double speed) {
        sparkMaxA.set(speed);
        sparkMaxB.set(speed); 
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

            pidController.setPID(kP, kI, kD);
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
            pidController.setPID(p, i, d);
        });
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

    /**
     * Test command to verify subsystem is working, recommended use is for autonomous command testing
     */
    public Command test() {
        return runOnce(() -> {
            System.out.println("Hi");
        });
    }
}
