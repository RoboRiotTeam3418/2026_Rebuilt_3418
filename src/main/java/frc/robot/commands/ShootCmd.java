package frc.robot.commands;

import com.revrobotics.spark.SparkBase.ControlType;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Feeder;
import frc.robot.subsystems.ShooterSubsystem;

public class ShootCmd extends Command {
    private ShooterSubsystem shooter;
    private Feeder feeder;
    private double setpoint = 1100;

    /**
    * The shoot command, shoots balls.
    *
    * @param shooterSubsystem The shooter subsystem.
    */
    public ShootCmd(ShooterSubsystem shooterSubsystem, Feeder feeder) { // Sets everything up
        this.shooter = shooterSubsystem;
        addRequirements(shooterSubsystem);
        addRequirements(feeder);
    }

    public ShootCmd(ShooterSubsystem shooterSubsystem, Feeder feeder, double setpoint) { // Sets everything up
        this.shooter = shooterSubsystem;
        this.setpoint = setpoint;
        addRequirements(shooterSubsystem);
        addRequirements(feeder);
    }

    @Override
    public void initialize() {
        System.out.println("Running autonomous shoot command...");
    }

    @Override
    public void execute() {
        shooter.pidControllerA.setSetpoint(setpoint, ControlType.kVelocity);
        if (shooter.shoudFeed(setpoint)) {
            feeder.feedBalls();
        } else {
            feeder.stopFeeding();
        }
    }

    @Override
    public void end(boolean interrupted) {
        shooter.setSpeeds(0);
        feeder.stopFeeding();
        System.out.println("Finished shoot command.");
    }
}
