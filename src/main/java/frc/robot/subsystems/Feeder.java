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
        feeder.set(0.45);
    }

    public Command stopFeeding() {
        return runOnce(() -> {
            feeder.set(0);
        });
    }

    public Command feed() {
        return runOnce(() -> {
            /*Thread t = new Thread() {
                @Override
                public void run() {
                    try {
                        Thread.sleep(750);
                        feedBalls();
                    } catch (InterruptedException e) {
                        e.printStackTrace();
                    }
                }
            };

            t.start();*/
            feedBalls();
            
            });//.finallyDo(() -> {stopFeeding();});
    }
}
