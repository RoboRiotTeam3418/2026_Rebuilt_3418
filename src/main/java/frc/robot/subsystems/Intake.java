package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax; // Although we don't know what exact type of motors we will be using,
                                                       // It is likely that we will be using SparkMaxes.


public class Intake extends SubsystemBase {
    private PWMSparkMax motorA; 
    private PWMSparkMax motorB;

    public Intake() { // Constructor
        motorA = new PWMSparkMax(0);
        motorB = new PWMSparkMax(0);
    }

    /**
     * This subsystem should have its default command set to this command with a Speed of 0. 
     * This probably should be done in RobotContainer.
     */

    public Command intakeCommand(double Speed) { // The intake is a lot simplier compared to the other subsystems (probably).
        return run(() -> {                       // As such, I am not sure if using an "actual command" is neccessary.
            motorA.set(Speed);
            motorB.set(-Speed); // Probably opposites
        });
    }

    /**
   * An example method querying a boolean state of the subsystem (for example, a digital sensor).
   *
   * @return value of some boolean subsystem state, such as a digital sensor.
   */
  public boolean exampleCondition() {
    // Query some boolean state, such as a digital sensor.
    return false;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}


