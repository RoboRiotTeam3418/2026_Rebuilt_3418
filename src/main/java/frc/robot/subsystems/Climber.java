package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.SubsystemConstants;
import frc.robot.util.math.DeadbandUtils;

public class Climber extends SubsystemBase{
    SparkMax climb1;
    DigitalInput bottom;
    double bottomPos=-32;

    public Climber() {
        climb1=new SparkMax(SubsystemConstants.CLIMBER_MOTOR, MotorType.kBrushless);
        bottom=new DigitalInput(0);
    }

    public void climb(double speed) {
        if ((speed < 0 && bottom.get())||(speed>0 && getMotorPos()>bottomPos+32)) {
            speed = 0;
        } else {
        climb1.set(speed);
        }
    }
    @Override
    public void periodic() {
        if (bottom.get()) {
            bottomPos=getMotorPos();
        }
    }
    //2 means at top, 0 means at bottom, 1 means in motion
    public int getHeight() {
        if (DeadbandUtils.isWithin(getMotorPos(), bottomPos+32, 2)) {
            return 2;
        } else if (bottom.get()) {
            return 0;
        }
        return 1;
    }
    public double getMotorPos() {
        return climb1.getEncoder().getPosition();
    }
    public double getCurrentDirection() {
        return climb1.get();
    }
    public boolean atBottom() {
        return bottom.get();
    }
    }
