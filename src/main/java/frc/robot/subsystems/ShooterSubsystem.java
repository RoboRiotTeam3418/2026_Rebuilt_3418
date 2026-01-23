package frc.robot.subsystems;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ShooterSubsystem extends SubsystemBase {
    public static ShooterSubsystem Instance;

    public double p, i, d;
    public PIDController pidController;

    public ShooterSubsystem() {
        Instance = this;

        if (DriverStation.isTestEnabled()) {
            System.out.println("Shooter subsystem loading...");
        }

        pidController = new PIDController(p, i, d);
        pidController.setSetpoint(1);
    }

    public Command Shoot() {
        return run(() -> {
            if (!pidController.atSetpoint()) {
                //pidController.calculate();
            }
        });
    }
}
