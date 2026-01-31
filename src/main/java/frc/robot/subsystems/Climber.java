package frc.robot.subsystems;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Climber extends SubsystemBase{
    SparkMax rotMotor;
    SparkMax climbMotor;
    AbsoluteEncoder rotEncoder;

    public Climber() {
        rotMotor=new SparkMax(0, MotorType.kBrushless);
        climbMotor= new SparkMax(0, MotorType.kBrushless);
        rotEncoder=rotMotor.getAbsoluteEncoder();
    }

    public Command climb(double speed) {
        return runOnce(()->{climbMotor.set(speed);});
    }
}
