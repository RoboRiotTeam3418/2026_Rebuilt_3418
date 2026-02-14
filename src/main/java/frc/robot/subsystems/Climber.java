package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.SubsystemConstants;

public class Climber extends SubsystemBase{
    SparkMax climb1;
    DigitalInput top;
    DigitalInput bottom;

    public Climber() {
        climb1=new SparkMax(SubsystemConstants.CLIMBER_MOTOR, MotorType.kBrushless);
        top=new DigitalInput(0);
        bottom= new DigitalInput(1);
    }

    public void climb(double speed) {
        climb1.set(speed);
    }
    //2 means at top, 0 means at bottom, 1 means in motion
    public int getHeight() {
        if (top.get()) {
            return 2;
        } else if (bottom.get()) {
            return 0;
        }
        return 1;
    }
    public double getCurrentDirection() {
        return climb1.get();
    }
    }
