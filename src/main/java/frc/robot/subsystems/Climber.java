package frc.robot.subsystems;

import com.revrobotics.spark.SparkAnalogSensor;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Climber extends SubsystemBase{
    SparkMax climb1;
    SparkMax climb2;
    SparkAnalogSensor climbPot;

    public Climber() {
        climb1=new SparkMax(0, MotorType.kBrushless);
        climb2= new SparkMax(0, MotorType.kBrushless);
        climbPot=climb1.getAnalog();
    }

    public void climb(double speed) {
        climb1.set(speed);
        climb2.set(-speed);
    }
    //TODO get the proper positions of the string pot
    public double getCurrentHeight() {
        return climbPot.getPosition();
    }
}
