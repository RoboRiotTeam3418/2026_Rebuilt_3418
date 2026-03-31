package frc.robot.commands;

import com.revrobotics.spark.SparkBase.ControlType;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Feeder;
import frc.robot.subsystems.ShooterSubsystem;

public class FlywheelCmd extends Command {
    private ShooterSubsystem shooter;
    private double setpoint = 1600;

    /**
    * The shoot command, shoots balls.
    *
    * @param shooterSubsystem The shooter subsystem.
    */
    public FlywheelCmd(ShooterSubsystem shooterSubsystem) { // Sets everything up
        this.shooter = shooterSubsystem;
        addRequirements(shooterSubsystem);
    }


    @Override
    public void initialize() {
        System.out.println("Running autonomous shoot command...");
    }

    @Override
    public void execute() {
        shooter.setTargetSpeed(3500);
    }

    @Override
    public void end(boolean interrupted) {
        //shooter.setTargetSpeed(0);
        shooter.stopMotors();
        System.out.println("Finished shoot command.");
    }
}
