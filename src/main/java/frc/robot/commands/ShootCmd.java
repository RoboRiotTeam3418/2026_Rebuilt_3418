package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ShooterSubsystem;

public class ShootCmd extends Command {
    private ShooterSubsystem shooter;
    private double setpoint = -.7;
    private PIDController pid;

    /**
    * The shoot command, shoots balls.
    *
    * @param shooterSubsystem The shooter subsystem.
    */
    public ShootCmd(ShooterSubsystem shooterSubsystem) { // Sets everything up
        this.shooter = shooterSubsystem;
        addRequirements(shooterSubsystem);
    }

    @Override
    public void initialize() {
        //pid = shooter.pidController;
        pid.setSetpoint(setpoint);
    }

    @Override
    public void execute() {
        shooter.setSpeeds(pid.calculate(shooter.getSpeeds(), setpoint)); // This should never be ran at the same time as ShooterSubsystem, this is for auto use only
        SmartDashboard.putNumber("speed", pid.calculate(shooter.getSpeeds(), setpoint));
        SmartDashboard.putBoolean("Ready to Shoot?",shooter.ready().getAsBoolean());

    }

    @Override
    public void end(boolean interrupted) {
        shooter.setSpeeds(0);
    }
}
