package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.SubsystemConstants;

public class Feeder extends SubsystemBase {
    SparkMax feeder;

    public Feeder() {
        feeder = new SparkMax(SubsystemConstants.FEEDER_MOTOR, SparkMax.MotorType.kBrushless);
    }

    public void feedBalls() {
        feeder.set(0.5);
    }

    public void stopFeeding() {
        feeder.set(0);
    }

    public Command feed() {
        return run(() -> {feedBalls();}).finallyDo(() -> {stopFeeding();});
    }
}
