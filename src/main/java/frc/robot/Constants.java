// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;


import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import swervelib.math.Matter;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean
 * constants. This class should not be used for any other purpose. All constants
 * should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {

  public static class OperatorConstants {
    public static final CommandJoystick PRIMARY = new CommandJoystick(1);
    public static final CommandXboxController SECONDARY = new CommandXboxController(0);

    // Joystick Deadband
    public static final double DEADBAND = 0.15;
    public static final double LEFT_Y_DEADBAND = 0.15;
    public static final double RIGHT_X_DEADBAND = 0.15;
    public static final double TURN_CONSTANT = 6;
    public static final double THRUST_SCALAR = ((23.0 / 9.0) / (40.0 / 9.0))/2;
  }

  public static final class DrivebaseConstants {

    // Hold time on motor brakes when disabled
    public static final double WHEEL_LOCK_TIME = 10; // seconds
  }

  /** The mass of the robot */
  public static final double ROBOT_MASS = 86.08; // 32lbs * kg per pound  new weight is 86.08 lbs
  public static final Matter CHASSIS = new Matter(new Translation3d(0, 0, Units.inchesToMeters(8)), ROBOT_MASS);
  public static final double LOOP_TIME = 0.02; // s, 20ms + 110ms sprk max velocity lag
  public static final double MAX_SPEED = Units.feetToMeters(14.5);
  // Maximum speed of the robot in meters per second, used to limit acceleration.
  public static final double DONT_SEE_TAG_WAIT_TIME = 1;
  public static final double POSE_VALIDATION_TIME = 0.3;

  /** The limelight pipeline id to be used with the limelight */
  public static final int LIMELIGHT_PIPELINE_ID = 0;
  /** Disables apriltag tracking :( */
  public static final boolean SAD_LIMELIGHT_MODE = false; //True for testing

  public static final class SubsystemConstants {
    // Shooter IDs
    public static final int SHOOTER_MOTOR_A = 14;
    public static final int SHOOTER_MOTOR_B = 15;
    public static final int FEEDER_MOTOR = 16;

    //Climber ID
    public static final int CLIMBER_MOTOR=12;

    // Intake IDs
    public static final int INTAKEPIVOTID = 17;
    public static final int INTAKEID = 18;
    public static final double INTAKE_MAX_ANGLE_IN = 0; // Maximum angle of intake based on interior of robot (placeholder)
    public static final double INTAKE_MAX_ANGLE_OUT = 0; // Maximum angle of intake based on exterior of robot (placeholder)
    public static final double INTAKE_PIVOT_SPEED = 0.5; // Constant pivot speed

    // Hopper IDs
    public static final int HOPPER_AGITATORS = 19;
    public static final int HOPPER_BOUNCER = 20;
  }

  public static final class AprilTagConstants {


    // IDs
    public static final int HUB_CENTER_BLUE = 10;
    public static final int HUB_CENTER_RED = 26;
  }
}
