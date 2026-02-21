package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Climber;

public class ManualClimbCmd extends Command {
    private Climber climb;
    private double moveSpeed;

    public ManualClimbCmd(Climber subsystem,double speed) { // Sets everything up
        this.climb = subsystem;
        this.moveSpeed = speed;
        addRequirements(subsystem);
    }

    @Override//if at the bottom .2, if at the top -.2 if in the middle keeps moving
    public void initialize() {

    }

    @Override
    public void execute() {
        climb.climb(moveSpeed);
    }
    @Override//if it's at the right point
    public boolean isFinished() {
        return false;
    }
    @Override
    public void end(boolean interrupted) {
        climb.climb(0);
    }
}
