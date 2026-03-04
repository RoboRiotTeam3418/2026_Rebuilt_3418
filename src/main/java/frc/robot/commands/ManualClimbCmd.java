/**THIS IS REDUNDANT AND NOT TO BE USED OUTSIDE OF TESTING USE 
 * USE {@link ClimbingCmd}
 */
package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.SubsystemConstants;
import frc.robot.subsystems.Climber;

public class ManualClimbCmd extends Command {
    private Climber climb;
    private double moveSpeed;
    private boolean shouldEnd;

    public ManualClimbCmd(Climber subsystem,double speed, boolean auto) { // Sets everything up
        this.climb = subsystem;
        this.moveSpeed = speed;
        this.shouldEnd = auto;
        addRequirements(subsystem);
    }

    @Override//if at the bottom .2, if at the top -.2 if in the middle keeps moving
    public void initialize() {

    }

    @Override
    public void execute() {
        if (shouldEnd) {
            if (climb.getMotorPos()>climb.getBottom()+SubsystemConstants.CLIMBER_DISTANCE&&moveSpeed>0) {
            climb.climb(0);
        } else if (climb.atBottom()&&moveSpeed<0) {
            climb.climb(0);
        } else {
        climb.climb(moveSpeed);
        }
        } else {
            climb.climb(moveSpeed);
        }
        System.out.println(climb.getMotorPos());
    }
    @Override//if it's at the right point
    public boolean isFinished() {
        return (shouldEnd&&((climb.getMotorPos()>climb.getBottom()+SubsystemConstants.CLIMBER_DISTANCE&&moveSpeed>0)||(climb.atBottom()&&moveSpeed<0)));
    }
    @Override
    public void end(boolean interrupted) {
        climb.climb(0);
    }
}
