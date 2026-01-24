package frc.robot.subsystems;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants;
import frc.robot.Constants.OperatorConstants;
import frc.robot.util.drivers.LimelightHelpers;
import frc.robot.util.drivers.LimelightHelpers.LimelightTarget_Fiducial;
import frc.robot.util.drivers.LimelightHelpers.RawFiducial;
import frc.robot.util.math.MathUtils;
import swervelib.SwerveInputStream;

public class ShooterSubsystem extends SubsystemBase {
    public static ShooterSubsystem Instance;
    CommandJoystick m_primary = Constants.OperatorConstants.PRIMARY;
    CommandXboxController m_secondary = Constants.OperatorConstants.SECONDARY;
    private SwerveSubsystem drivebase;

    public double p, i, d;
    public PIDController pidController;

    public ShooterSubsystem(SwerveSubsystem drivebase) {
        Instance = this;

        Log("Shooter subsystem loading...");
        Log("P: " + p + ", I: " + i + ", D: " + d);
        pidController = new PIDController(p, i, d);
        pidController.setSetpoint(1);
    }

    public DoubleSupplier aprilTagPos = () -> {
        if (!LimelightHelpers.getTV("limelight")) return 0;
        for (RawFiducial target : LimelightHelpers.getRawFiducials("limelight")) {
            if (target.id == 10 || target.id == 25) {
                return MathUtils.clamp(target.txnc, -0.8, 0.8);
            }
        }

        return 0;
    };

    SwerveInputStream driveAngularVelocity = SwerveInputStream.of(drivebase.getSwerveDrive(),
      () -> m_primary.getY() * ((m_primary.getZ() - (23.0 / 9.0)) / (40.0 / 9.0)),
      () -> m_primary.getX() * ((m_primary.getZ() - (23.0 / 9.0)) / (40.0 / 9.0)))
      .withControllerRotationAxis(aprilTagPos)
      .deadband(OperatorConstants.DEADBAND)
      .allianceRelativeControl(true);

    public double limelightCalculator() {
        if (!LimelightHelpers.getTV("limelight")) return 0.85; // set flywheel speed regardless of vision
        for (RawFiducial target : LimelightHelpers.getRawFiducials("limelight")) {
            if (target.id == 10 || target.id == 25) {
                return MathUtils.clamp((1 / target.ta), 0.05, 1);
            }
        }


        return 0.85;
    }

    public Command Shoot() {
        return run(() -> {
            // motor.set(pid.calculate(encoder.getDistance(), limelightCalculator())); this is pretty much everything
            // feed balls here
            drivebase.driveFieldOriented(driveAngularVelocity);
        });
    }
    public Command UpdatePID(double kP, double kI, double kD) {
        return runOnce(() -> {
            pidController.setPID(kP, kI, kD);
            p = kP;
            i = kI;
            d = kD;
        });
    }

    public void Log(Object objects) {
        if (DriverStation.isTestEnabled()) {
            System.out.println(objects);
        }
    }
}
