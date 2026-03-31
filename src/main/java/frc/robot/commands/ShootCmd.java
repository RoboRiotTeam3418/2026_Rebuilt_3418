package frc.robot.commands;

import com.revrobotics.spark.SparkBase.ControlType;

import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Feeder;
import frc.robot.subsystems.Servos;
import frc.robot.subsystems.ShooterSubsystem;

public class ShootCmd extends Command {
    private ShooterSubsystem shooter;
    private Feeder feeder;
    private double setpoint = 1600;
    private double angle;
    private Servos servos;

    /**
    * The shoot command, shoots balls.
    *
    * @param shooterSubsystem The shooter subsystem.
    */
    public ShootCmd(ShooterSubsystem shooterSubsystem, Feeder feeder) { // Sets everything up
        this.shooter = shooterSubsystem;
        this.feeder = feeder;
        addRequirements(shooterSubsystem);
        addRequirements(feeder);
    }

    public ShootCmd(ShooterSubsystem shooterSubsystem, Feeder feeder, double setpoint,double angle, Servos servos) { // Sets everything up
        this.shooter = shooterSubsystem;
        this.setpoint = setpoint;
        this.feeder = feeder;
        this.angle = angle;
        this.servos=servos;
        addRequirements(shooterSubsystem,servos);
        addRequirements(feeder);
    }

    @Override
    public void initialize() {
        System.out.println("Running autonomous shoot command...");
        servos.setAngle(angle);
    }

    @Override
    public void execute() {
        shooter.setTargetSpeed(setpoint);

        if (shooter.ready().getAsBoolean()) {
            feeder.feedBalls();
        } else {
            feeder.stopFeedBalls();
        }
    }

    @Override
    public void end(boolean interrupted) {
        //shooter.setTargetSpeed(0);
        feeder.stopFeedBalls();
        shooter.stopMotors();
        System.out.println("Finished shoot command.");
    }
}
