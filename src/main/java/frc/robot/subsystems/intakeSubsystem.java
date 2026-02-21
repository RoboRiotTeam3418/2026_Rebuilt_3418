package frc.robot.subsystems;

// Software
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

// Hardware (software)
import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;


public class IntakeSubsystem extends SubsystemBase {
    // Pivot
    public SparkMax pivotMotor; // needs to be public for commands to access
    public AbsoluteEncoder ThroughboreEncoder; // needs to be public for commands to access

    // Intake
    private SparkMax IntakeMotor; // This probably doesn't need to be public.
    private AbsoluteEncoder iMEncoder; // Are there any software differences between the throughbore encoder and the built in encoder?

    //Constants (maybe move over to Constants.java later)
    public final double MAX_ANGLE_IN = 0; // Maximum angle of intake based on interior of robot (placeholder value)
    public final double MAX_ANGLE_OUT = 10; // Maximum angle of intake based on exterior of robot (placeholder value)
    public final double pivotSpeed = 0.05; // Constant pivot  (Set to a very low value for testing)

    // Constructor
    public IntakeSubsystem() {
        pivotMotor = new SparkMax(Constants.SubsystemConstants.INTAKEPIVOTID, MotorType.kBrushless);
        ThroughboreEncoder = pivotMotor.getAbsoluteEncoder();

        IntakeMotor = new SparkMax(Constants.SubsystemConstants.INTAKEID, MotorType.kBrushless);
        iMEncoder = IntakeMotor.getAbsoluteEncoder();
    }

    /*
     Note: The pivot is much more complex than the intake itself. As this is the case, the functionality of the intake
           is in this subsystem rather than being its own separate command.

           This also means that the command that controls the pivot of the intake is in its own separate file (in commands).
           you probably already saw it though.
    */
    
    public Command intakeCmd(double speed) { // Run the intake
        return run(() -> {                       
            IntakeMotor.set(speed);
        });
    }

    public Command intakeResetCommand() { // Don't bind this command to any trigger.

    /**
     * This subsystem should have its default command set to this command.
     * This probably should be done in RobotContainer.
     */

      return run(() -> {
        IntakeMotor.set(0);
        if (ThroughboreEncoder.getPosition() > MAX_ANGLE_IN){
          if (ThroughboreEncoder.getPosition() < MAX_ANGLE_IN +10){
            pivotMotor.set(-pivotSpeed/2);
          } else {
            pivotMotor.set(-pivotSpeed);
          }
        }
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


