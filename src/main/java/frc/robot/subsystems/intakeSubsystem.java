package frc.robot.subsystems;

// Software
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.SubsystemConstants;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

// Hardware (software)
import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;


public class intakeSubsystem extends SubsystemBase {
    // Pivot
    private SparkMax pivotMotor; 
    private AbsoluteEncoder ThroughboreEncoder;
    private SparkFlex innerMotor;

    // Intake
    private SparkMax IntakeMotor;


    public intakeSubsystem() { // Constructor
        pivotMotor = new SparkMax(SubsystemConstants.INTAKEPIVOTID, MotorType.kBrushless); // Placeholder ID and Placeholder MotorType
        ThroughboreEncoder = pivotMotor.getAbsoluteEncoder();
        innerMotor = new SparkFlex(SubsystemConstants.INNERID, MotorType.kBrushless);
        IntakeMotor = new SparkMax(SubsystemConstants.INTAKEID, MotorType.kBrushless); // Placeholder ID and Placeholder MotorType
    }


    // --------------- methods/functions --------------- //

    // ---------- active / commands ---------- //
    public Command setIntakeSPD(double speed) { // method to activate intake
      return run(() -> {                       
        IntakeMotor.set(speed);
      });
    }

    public Command setPivotSPD(double speed) { // method to pivot the intake
      return run(() -> {
        // Slows down when closer to soft stops or prevents movement when exceeding or too close to soft stops
       /* if ((ThroughboreEncoder.getPosition() > SubsystemConstants.INTAKE_MAX_ANGLE_IN&&speed<0) || (ThroughboreEncoder.getPosition() < SubsystemConstants.INTAKE_MAX_ANGLE_OUT&&speed>0)){
          pivotMotor.set(0);
        } else if (ThroughboreEncoder.getPosition() > (SubsystemConstants.INTAKE_MAX_ANGLE_IN - 0.075) || ThroughboreEncoder.getPosition() < (SubsystemConstants.INTAKE_MAX_ANGLE_OUT + 0.075)){
          pivotMotor.set(speed/2);
        } else {*/
          pivotMotor.set(speed);
        //}
      });
    }
    public void pivot(double spd) {
      pivotMotor.set(spd);
      innerMotor.set(spd);
    }

    public Command SetPivotSpeed_NoStops(double speed) { // Not reccomended to use, currently for "testing" purposes. Will remove later.
      return run(() -> {                       
        IntakeMotor.set(speed);
      });
    }

    public Command resetIntake(double speed) { // a test command (probably won't need this)
      return run(() -> {                       
        pivotMotor.set(speed);
      });
    }

    
    // ---------- informational ---------- //
    public double getPivotEncoderPos() {
      return ThroughboreEncoder.getPosition();
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
    // as of now, this is temporary since we dont have hopper ai mentor assistant.
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}