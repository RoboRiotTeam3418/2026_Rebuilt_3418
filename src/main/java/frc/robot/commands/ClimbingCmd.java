package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Climber;

public class ClimbingCmd extends Command {
    private Climber climb;
    private double direction;

    public ClimbingCmd(Climber subsystem) { // Sets everything up
        this.climb = subsystem;
        addRequirements(subsystem);
    }

    @Override//if at the bottom .2, if at the top -.2 if in the middle keeps moving
    public void initialize() {
        if (climb.getHeight()!=1){
            direction=-(climb.getHeight()-1)/5;
        } else {
            direction = climb.getCurrentDirection();
        }
    }

    @Override
    public void execute() {
        climb.climb(direction);
    }
    @Override//if it's at the right point
    public boolean isFinished() {
        return climb.getHeight()-1==direction*5;
    }
    @Override
    public void end(boolean interrupted) {
        climb.climb(0);
    }
}
