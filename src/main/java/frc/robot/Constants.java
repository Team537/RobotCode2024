// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.util.Units;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static class OperatorConstants {
    public static final int kDriverControllerPort = 0;
  }

  public static final class DriveConstants {

    public static final double MAX_DIVE_SPEED_METERS_PER_SECOND = 100;

    
    // Swerve module constants
    // NOTE: All of these values are currently placeholders. Please replace them with the actual values when possible.
    public static final int FRONT_RIGHT_DRIVE_MOTOR_PORT = 1;
    public static final int FRONT_LEFT_DRIVE_MOTOR_PORT  = 2;
    public static final int BACK_RIGHT_DRIVE_MOTOR_PORT  = 3;
    public static final int BACK_LEFT_DRIVE_MOTOR_PORT   = 4;

    public static final int FRONT_RIGHT_TURNING_MOTOR_PORT = 5;
    public static final int FRONT_LEFT_TURNING_MOTOR_PORT  = 6;
    public static final int BACK_RIGHT_TURNING_MOTOR_PORT  = 7;
    public static final int BACK_LEFT_TURNING_MOTOR_PORT   = 8;

    public static final boolean FRONT_RIGHT_DRIVE_MOTOR_REVERSED = false;
    public static final boolean FRONT_LEFT_DRIVE_MOTOR_REVERSED  = true;
    public static final boolean BACK_RIGHT_DRIVE_MOTOR_REVERSED  = false;
    public static final boolean BACK_LEFT_DRIVE_MOTOR_REVERSED   = true;

    public static final boolean FRONT_RIGHT_TURNING_MOTOR_REVERSED = true;
    public static final boolean FRONT_LEFT_TURNING_MOTOR_REVERSED  = true;
    public static final boolean BACK_RIGHT_TURNING_MOTOR_REVERSED  = false;
    public static final boolean BACK_LEFT_TURNING_MOTOR_REVERSED   = true;

    
  }

  /**
   * This class contains all of the constant values used in the SwerveModule class. 
   */
  public static final class SwerveModuleConstants {

    // Hardware settings
    public static final double WHEEL_DIAMETER_METERS  = Units.inchesToMeters(3);
    public static final double DRIVE_MOTOR_GEAR_RATIO = 5; // Value is currently a placeholder.
    public static final double TURN_MOTOR_GEAR_RATIO  = 10; // Value is currently a placeholder.

    // Calculate all of the desired conversion values
    public static final double DRIVE_MOTOR_TICKS_PER_METER = 5;

    // PID Values
    public static final double kp = 100;
    public static final double ki = 75;
    public static final double kd = 1;

  }
}
