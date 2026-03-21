package frc.robot.subsystems;
import frc.robot.Constants;

// Software
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

// Hardware (software)
//import com.revrobotics.spark.SparkMax; I don't believe the Neo vortex uses a sparkmax
import com.revrobotics.spark.SparkFlex;

import com.revrobotics.spark.SparkLowLevel.MotorType;


public class Hopper extends SubsystemBase {
    // Objects
    // A singular NEO vortex controls the belt / conveyor
    SparkFlex conveyerMotor;
    // private SparkMax actuatorMotor;
    //private SparkMax bouncerMotor;

    // Variables
    private double constantSpeed = 0.5;
    //private double actSpeed = 0.25;
    //private double bounceSpeed = 0.5;
    

    public Hopper() { // Constryuctor
        conveyerMotor = new SparkFlex(0, MotorType.kBrushless);
        //actuatorMotor = new SparkMax(0, MotorType.kBrushless);
        //bouncerMotor = new SparkMax(Constants.SubsystemConstants.HOPPER_BOUNCER, MotorType.kBrushless);
    }

// !!! Reduntant code moved to bottom!!!

    public Command runIndex(){
      return run(() -> {
        conveyerMotor.set(constantSpeed);
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


// removed from subsystem

// probably won't need this in the future, but it's commented just in case.
    /*
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
    */

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