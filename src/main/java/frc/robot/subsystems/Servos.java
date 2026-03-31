package frc.robot.subsystems;

import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.SubsystemConstants;

public class Servos extends SubsystemBase {
    Servo leftServo, rightServo;
    public Servos() {
        //System.out.println("This should be true: " + (Servos != null));
        leftServo = new Servo(SubsystemConstants.LEFT_SERVO_ID);
        rightServo = new Servo(SubsystemConstants.RIGHT_SERVO_ID);
    }

    public void setAngle(double position) {
        leftServo.setAngle(position);
        rightServo.setAngle(180-position);
        SmartDashboard.putNumber("Servo angle", position);
    }
    public Command setAngles(double angle) {
        return runOnce(()-> {
            System.out.println(angle);
            setAngle(angle);
        });
    }
}
