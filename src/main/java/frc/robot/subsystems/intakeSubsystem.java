package frc.robot.subsystems;

// Software
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.SubsystemConstants;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

// Hardware (software)
import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
//import edu.wpi.first.wpilibj.Encoder; (not used?)
//import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax; (also not used)


public class intakeSubsystem extends SubsystemBase {
    // Pivot
    public SparkMax pivotMotor; 
    public AbsoluteEncoder ThroughboreEncoder;

    // Intake
    private SparkMax IntakeMotor; // This probably doesn't need to be public.

    // Constructor
    public intakeSubsystem() {
        pivotMotor = new SparkMax(SubsystemConstants.INTAKEPIVOTID, MotorType.kBrushless); // Placeholder ID and Placeholder MotorType
        ThroughboreEncoder = pivotMotor.getAbsoluteEncoder();

        IntakeMotor = new SparkMax(SubsystemConstants.INTAKEID, MotorType.kBrushless); // Placeholder ID and Placeholder MotorType
    }

    /*
     Note: The pivot is much more complex than the intake itself. As this is the case, the functionality of the intake
           is in this subsystem rather than being its own separate command.

           This also means that the command that controls the pivot of the intake is in its own separate file (in commands).
           you probably already saw it though.
    */
    
    public Command intake(double speed) {
    /**
     * This subsystem should have its default command set to this command with a Speed of 0. 
     * This probably should be done in RobotContainer.
     */
        return run(() -> {                       
            IntakeMotor.set(speed);
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
    // good info (tasty)
    SmartDashboard.putNumber("intakePivot Encoder Angle", ThroughboreEncoder.getPosition()); // might rename this (dunno)

  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}


