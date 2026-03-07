package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.SubsystemConstants;
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
        if (climb.getStart()) {
            direction=-.2;
            climb.setStart(false);
            System.out.println("test");
        } else if (climb.getHeight()==2){
            direction=-.2;
        }else if (climb.getHeight()==0){
            direction=.2;
        } else {
            direction = -climb.getCurrentDirection();
        }
    }

    @Override
    public void execute() {
        climb.climb(direction);
        SmartDashboard.putNumber("direction", direction);
    }
    @Override
    //if it's at the right point
    public boolean isFinished() {
        return ((climb.getMotorPos()>climb.getBottom()+SubsystemConstants.CLIMBER_DISTANCE&&direction>0)||(climb.atBottom()&&direction<0));
    }
    @Override
    public void end(boolean interrupted) {
        climb.climb(0);
    }
}
