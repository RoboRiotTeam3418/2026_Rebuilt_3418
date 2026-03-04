package frc.robot.subsystems;
import frc.robot.Constants;

// Software
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

// Hardware (software)
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;


public class Hopper extends SubsystemBase {
    // Objects
    private SparkMax actuatorMotor;
    //private SparkMax bouncerMotor;

    // Variables
    private double actSpeed = 0.25;
    //private double bounceSpeed = 0.5;
    

    public Hopper() { // Constructor
        actuatorMotor = new SparkMax(0, MotorType.kBrushless);
        //bouncerMotor = new SparkMax(Constants.SubsystemConstants.HOPPER_BOUNCER, MotorType.kBrushless);
    }

    public Command actuateOnly(Boolean Reversed, Boolean Stopped) { // true will cause motor to spin backwards
        return run(() -> { 
          if (Stopped == true){
            actuatorMotor.set(0);
          } else {
            if (Reversed == true){
                actuatorMotor.set(-actSpeed); // It is probably uneccessary to change this value, as such, it is hardcoded.
            } else {
                actuatorMotor.set(actSpeed); // It is probably uneccessary to change this value, as such, it is hardcoded.
            } 
          }               
        });
    }

    /* -- no bouncy bouncy yet
        public Command bounceOnly(Boolean Stopped) { // activates bouncer
        return run(() -> { 
          if (Stopped == true){
            bouncerMotor.set(0);
          } else {
            bouncerMotor.set(bounceSpeed); // It is probably uneccessary to change this value, as such, it is hardcoded.
          }               
        });
    }
    */

    /* -- no bouncy bouncy yet
        public Command activateHopper(Boolean Reversed, Boolean Stopped) { // true will cause motor to spin backwards
        return run(() -> { 
          if (Stopped == true){
            actuatorMotor.set(0);
            bouncerMotor.set(0);
          } else {
            bouncerMotor.set(bounceSpeed); // It is probably uneccessary to change this value, as such, it is hardcoded.

            if (Reversed == true){
                actuatorMotor.set(-actSpeed); // It is probably uneccessary to change this value, as such, it is hardcoded.
            } else {
                actuatorMotor.set(actSpeed); // It is probably uneccessary to change this value, as such, it is hardcoded.
            } 
          }               
        });
    }
    */


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
    actuatorMotor.set(0);
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}