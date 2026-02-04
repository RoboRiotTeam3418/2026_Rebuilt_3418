package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.ShooterSubsystem;

public class ShootCmd extends Command {
    private ShooterSubsystem shooter;
    private double setpoint = 0.7;
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
        pid = new PIDController(shooter.p, shooter.i, shooter.d);
        pid.setSetpoint(setpoint);
    }

    @Override
    public void execute() {
        shooter.setSpeeds(pid.calculate(shooter.encoderA.getVelocity() / Constants.MAX_NEO_VORTEX_SPEED, setpoint) * 10);
    }

    // I should add finish logic but idk.
}
