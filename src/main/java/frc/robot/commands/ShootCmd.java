package frc.robot.commands;

import com.revrobotics.spark.SparkBase.ControlType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
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
    public ShootCmd(ShooterSubsystem shooterSubsystem) { // Sets everything up
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
        shooter.pidControllerA.setSetpoint(setpoint, ControlType.kVelocity);
    }

    @Override
    public void execute() {
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
    }
}
