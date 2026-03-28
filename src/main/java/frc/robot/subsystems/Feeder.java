package frc.robot.subsystems;

import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.SubsystemConstants;

public class Feeder extends SubsystemBase {
    SparkMax feeder;
    private SparkFlex innerMotor;
    public Feeder() {
        feeder = new SparkMax(SubsystemConstants.FEEDER_MOTOR, SparkMax.MotorType.kBrushless);
        innerMotor = new SparkFlex(SubsystemConstants.INNERID, MotorType.kBrushless);
        //System.out.println("This should be true: " + (feeder != null));
    }

    public void feedBalls() {
        feeder.set(0.7);
        innerMotor.set(-.8);
    }

    public void stopFeedBalls() {
        feeder.set(0);
        innerMotor.set(0);
    }

    public Command stopFeeding() {
        return runOnce(() -> {
            stopFeedBalls();
        });
    }

    public Command feed() {
        return runOnce(() -> {
            feedBalls();
            
            });
    }
}
