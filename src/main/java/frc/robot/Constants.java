// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.List;

import com.revrobotics.CANSparkBase.IdleMode;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;

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
    public static final class DriveConstants {
        // Driving Parameters - Note that these are not the maximum capable speeds of
        // the robot, rather the allowed maximum speeds
        public static final double MAX_SPEED_METERS_PER_SECOND = 2.0; // 2 // meters per second
        public static final double BOOST_MODE_MAX_SPEED_METERS_PER_SECOND = 4.8; // meters per second
        public static final double MAX_ANGULAR_SPEED = 2 * Math.PI; // radians per second

        public static final double DIRECTION_SLEW_RATE = 2; // radians per second
        public static final double GRIP_DIRECTION_SLEW_RATE = 5; // radians per second
        public static final double BOOST_MODE_DIRECTION_SLEW_RATE = 1.8; // radians per second
        public static final double GRIP_BOOST_MODE_DIRECTION_SLEW_RATE = 5; // radians per second
        public static final double MAGNITUDE_POSITIVE_SLEW_RATE = 3.6; // percent per second (1 = 100%)
        public static final double MAGNITUDE_NEGATIVE_SLEW_RATE = 4.8; // percent per second (1 = 100%)
        public static final double GRIP_MAGNITUDE_POSITIVE_SLEW_RATE = 7.2; // percent per second (1 = 100%)
        public static final double GRIP_MAGNITUDE_NEGATIVE_SLEW_RATE = 6; // percent per second (1 = 100%)
        public static final double BOOST_MODE_MAGNITUDE_POSITIVE_SLEW_RATE = 2; // percent per second (1 = 100%)
        public static final double BOOST_MODE_MAGNITUDE_NEGATIVE_SLEW_RATE = 3.6; // percent per second (1 = 100%)
        public static final double ROTATIONAL_SLEW_RATE = 2.20; // percent per second (1 = 100%)

        // Chassis configuration
        public static final double TRACK_WIDTH = Units.inchesToMeters(16); // 16 for amy; 14.5 for alucard
        // Distance between centers of right and left wheels on robot
        public static final double WHEEL_BASE = Units.inchesToMeters(16); // 16 for amy; 14.5 for alucard
        // Distance between front and back wheels on robot
        public static final SwerveDriveKinematics DRIVE_KINEMATICS = new SwerveDriveKinematics(
                new Translation2d(WHEEL_BASE / 2, TRACK_WIDTH / 2),
                new Translation2d(WHEEL_BASE / 2, -TRACK_WIDTH / 2),
                new Translation2d(-WHEEL_BASE / 2, TRACK_WIDTH / 2),
                new Translation2d(-WHEEL_BASE / 2, -TRACK_WIDTH / 2));

        // Angular offsets of the modules relative to the chassis in radians
        public static final double FRONT_LEFT_CHASSIS_ANGULAR_OFFSET = -Math.PI / 2;
        public static final double FRONT_RIGHT_CHASSIS_ANGULAR_OFFSET = 0;
        public static final double BACK_LEFT_CHASSIS_ANGULAR_OFFSET = Math.PI;
        public static final double BACK_RIGHT_CHASSIS_ANGULAR_OFFSET = Math.PI / 2;

        // SPARK MAX CAN IDs
        public static final int FRONT_LEFT_DRIVE_CAN_ID = 2;
        public static final int BACK_LEFT_DRIVE_CAN_ID = 1;
        public static final int FRONT_RIGHT_DRIVE_CAN_ID = 7;
        public static final int BACK_RIGHT_DRIVE_CAN_ID = 8;

        public static final int FRONT_LEFT_TURNING_CAN_ID = 3;
        public static final int BACK_LEFT_TURNING_CAN_ID = 10;
        public static final int FRONT_RIGHT_TURNING_CAN_ID = 6;
        public static final int BACK_RIGHT_TURNING_CAN_ID = 9;

        public static final boolean GYRO_REVERSED = false;

        // orientation lock PID values
        public static final double ORIENTATION_LOCK_KP = 1;
        public static final double ORIENTATION_LOCK_KI = 0;
        public static final double ORIENTATION_LOCK_KD = 0.2;

    }

    public static final class ModuleConstants {
        // The MAXSwerve module can be configured with one of three pinion gears: 12T,
        // 13T, or 14T.
        // This changes the drive speed of the module (a pinion gear with more teeth
        // will result in a
        // robot that drives faster).
        public static final int DRIVING_MOTOR_PINION_TEETH = 14;

        // Invert the turning encoder, since the output shaft rotates in the opposite
        // direction of
        // the steering motor in the MAXSwerve Module.
        public static final boolean TURNING_ENCODER_INVERTED = true;

        // Calculations required for driving motor conversion factors and feed forward
        public static final double DRIVING_MOTOR_FREE_SPEED_RPS = NeoMotorConstants.FREE_SPEED_RPM / 60;
        public static final double WHEEL_DIAMETER_METERS = 0.06677;
        public static final double WHEEL_CIRCUMFERENCE_METERS = WHEEL_DIAMETER_METERS * Math.PI;
        // 45 teeth on the wheel's bevel gear, 22 teeth on the first-stage spur gear, 15
        // teeth on the bevel pinion
        public static final double DRIVING_MOTOR_REDUCTION = (45.0 * 22) / (DRIVING_MOTOR_PINION_TEETH * 15);
        public static final double DRIVE_WHEEL_FREE_SPEED_RPS = (DRIVING_MOTOR_FREE_SPEED_RPS
                * WHEEL_CIRCUMFERENCE_METERS)
                / DRIVING_MOTOR_REDUCTION;

        public static final double DRIVING_ENCODER_POSITIONAL_FACTOR = (WHEEL_DIAMETER_METERS * Math.PI)
                / DRIVING_MOTOR_REDUCTION; // meters
        public static final double DRIVING_ENCODER_VELOCITY_FACTOR = ((WHEEL_DIAMETER_METERS * Math.PI)
                / DRIVING_MOTOR_REDUCTION) / 60.0; // meters per second

        public static final double TURNING_ENCODER_POSITION_FACTOR = (2 * Math.PI); // radians
        public static final double TURNING_ENCODER_VELOCITY_FACTOR = (2 * Math.PI) / 60.0; // radians per second

        public static final double TURNING_ENCODER_POSITION_PID_MIN_INPUT = 0; // radians
        public static final double TURNING_ENCODER_POSITION_PID_MAX_INPUT = TURNING_ENCODER_POSITION_FACTOR; // radians

        public static final double DRIVING_KP = 0.04;
        public static final double DRIVING_KI = 0;
        public static final double DRIVING_KD = 0;
        public static final double DRIVING_FF = 1 / DRIVE_WHEEL_FREE_SPEED_RPS;
        public static final double DRIVING_MIN_OUTPUT = -1;
        public static final double DRIVING_MAX_OUTPUT = 1;

        public static final double TURNING_KP = 1;
        public static final double TURNING_KI = 0;
        public static final double TURNING_KD = 0;
        public static final double TURNING_FF = 0;
        public static final double TURNING_MIN_OUTPUT = -1;
        public static final double TURNING_MAX_OUTPUT = 1;

        public static final IdleMode DRIVING_MOTOR_IDLE_MODE = IdleMode.kBrake;
        public static final IdleMode TURNING_MOTOR_IDLE_MODE = IdleMode.kBrake;

        public static final int DRIVING_MOTOR_CURRENT_LIMIT = 50; // amps
        public static final int TURNING_MOTOR_CURRENT_LIMIT = 20; // amps
    }

    public static final class VisionConstants {

        // Data gathering settings
        public static final double SNAPSHOTS_PER_SECOND = 12; // How many snapshots are taken per second.
        public static final double SNAPSHOT_RATE = 1 / SNAPSHOTS_PER_SECOND;

        // Pipeline settings
        public static final int APRIL_TAG_PIPELINE = 0;
        public static final int OBJECT_DETECTION_PIPELINE = 1;
        public static final AprilTagFieldLayout APRIL_TAG_FIELD_LAYOUT = AprilTagFields.k2024Crescendo
                .loadAprilTagLayoutField();

        // Target settings
        public static final float MAX_AMBIGUITY = 0.2f; // The maximum amount of ambiguity allowed for detections.

        // Camera settings
        public static final String LIMELIGHT_NAME = "limelight";
        public static final String ARDUCAM_OV9281_USB_CAMERA_NAME = "Arducam_OV9281_USB_Camera";
        public static final String ARDUCAM_OV2311_USB_CAMERA_NAME = "Arducam_OV2311_USB_Camera";
        public static final String USB_2M_GS_CAMERA_NAME = "USB_2M_GS_camera";

        // Camera offsets (NOTE: These values are placeholders and thus subject to
        // change).
        public static final Transform3d ARDUCAM_OV9281_OFFSET = new Transform3d(
                new Translation3d(0, 0, 0),
                new Rotation3d(0, 0, 0));
        public static final Transform3d ARDUCAM_OV2311_OFFSET = new Transform3d(
                new Translation3d(0.1778, -0.2286, 0.3048),
                new Rotation3d(0, 0, 0));
        public static final Transform3d USB_2M_GS_CAMERA_OFFSET = new Transform3d(
                new Translation3d(-0.1016, -0.1143, 0.5588),
                new Rotation3d(0, 0, 0));
    }

    public static final class OIConstants {
        public static final int DRIVER_CONTROLLER_PORT = 0;
        public static final double DRIVE_DEADBAND = 0.1;
        public static final double INPUT_CURVE_POWER = 2.5;
    }

    public static final class AutoConstants {
        public static final double MAX_SPEED_METERS_PER_SECOND = 3;
        public static final double MAX_ACCELERATION_METERS_PER_SECOND_SQUARED = 3;
        public static final double MAX_ANGULAR_SPEED_RADIANS_PER_SECOND = Math.PI;
        public static final double MAX_ANGULAR_SPEED_RADIANS_PER_SECOND_SQUARED = Math.PI;

        public static final double X_CONTROLLER_KP = 1;
        public static final double X_CONTROLLER_KI = 0;
        public static final double X_CONTROLLER_KD = 0;

        public static final double Y_CONTROLLER_KP = 1;
        public static final double Y_CONTROLLER_KI = 0;
        public static final double Y_CONTROLLER_KD = 0;

        public static final double THETA_CONTROLLER_KP = 0.5;
        public static final double THETA_CONTROLLER_KI = 0;
        public static final double THETA_CONTROLLER_KD = 0.2;

        // Constraint for the motion profiled robot angle controller
        // Value is a float because xyz
        public static final double TRAJECTORY_THRESHOLD = 0.1; // the maximum distance away from a waypoint before the
                                                               // robot moves onto the next one (meters)
        public static final TrapezoidProfile.Constraints kThetaControllerConstraints = new TrapezoidProfile.Constraints(
                MAX_ANGULAR_SPEED_RADIANS_PER_SECOND, MAX_ANGULAR_SPEED_RADIANS_PER_SECOND_SQUARED);

        // Auto starting positions
        public static final Pose2d BLUE_1_STARTING_POSE = new Pose2d(1.629, 6.457, Rotation2d.fromDegrees(0));
        public static final Pose2d BLUE_2_STARTING_POSE = new Pose2d(1.278, 5.603, Rotation2d.fromDegrees(0));
        public static final Pose2d BLUE_3_STARTING_POSE = new Pose2d(1.629, 4.749, Rotation2d.fromDegrees(0));
        public static final Pose2d RED_1_STARTING_POSE = new Pose2d(14.884, 6.457, Rotation2d.fromDegrees(180));
        public static final Pose2d RED_2_STARTING_POSE = new Pose2d(15.235, 5.603, Rotation2d.fromDegrees(180));
        public static final Pose2d RED_3_STARTING_POSE = new Pose2d(14.884, 4.749, Rotation2d.fromDegrees(180));

        // Old paths
        public static final List<Pose2d> RED_1_TRAJECTORY = List.of(
                new Pose2d(15.036, 7.671, RED_1_STARTING_POSE.getRotation()),
                new Pose2d(15.072, 7.671, RED_1_STARTING_POSE.getRotation()));
        public static final List<Pose2d> RED_2_TRAJECTORY = List.of(
                new Pose2d(12.429, 5.713, RED_2_STARTING_POSE.getRotation()));
        public static final List<Pose2d> RED_3_TRAJECTORY = List.of(
                new Pose2d(14.940, 1.580, RED_3_STARTING_POSE.getRotation()),
                new Pose2d(13.475, 1.195, RED_3_STARTING_POSE.getRotation()));
        public static final List<Pose2d> BLUE_1_TRAJECTORY = List.of(
                new Pose2d(2.926, 7.563, BLUE_1_STARTING_POSE.getRotation()),
                new Pose2d(5.497, 7.563, BLUE_1_STARTING_POSE.getRotation()));
        public static final List<Pose2d> BLUE_2_TRAJECTORY = List.of(
                new Pose2d(5.293, 5.593, BLUE_2_STARTING_POSE.getRotation()));
        public static final List<Pose2d> BLUE_3_TRAJECTORY = List.of(
                new Pose2d(3.503, 0.931, BLUE_3_STARTING_POSE.getRotation()),
                new Pose2d(4.440, 0.931, BLUE_3_STARTING_POSE.getRotation()));

        // Blue 1 complex auto positions
        public static final Pose2d BLUE_1_SCORING_POSITION = new Pose2d(0.840,6.979, Rotation2d.fromDegrees(60));
        public static final Pose2d BLUE_1_NEAR_AMP_POSITION = new Pose2d(1.629, 7.807, Rotation2d.fromDegrees(0));
        public static final Pose2d BLUE_1_FIRST_NOTE_POSITION = new Pose2d(7.717, 7.436, Rotation2d.fromDegrees(0));
        public static final Pose2d BLUE_1_SECOND_NOTE_POSITION =  new Pose2d(7.717, 5.785, Rotation2d.fromDegrees(0));

        // Blue 2 complex auto positions
        public static final Pose2d BLUE_2_SCORING_POSITION = new Pose2d(1.418,5.603, Rotation2d.fromDegrees(0));
        public static final Pose2d BLUE_2_RIGHTMOST_NOTE_POSITION = new Pose2d(2.273, 4.088, Rotation2d.fromDegrees(0));
        public static final Pose2d BLUE_2_CENTER_NOTE_POSITION = new Pose2d(2.273, 5.569, Rotation2d.fromDegrees(0));
        public static final Pose2d BLUE_2_LEFTMOST_NOTE_POSITION =  new Pose2d(2.273, 7.004, Rotation2d.fromDegrees(0));

        // Blue 3 complex auto positions
        public static final Pose2d BLUE_3_SCORING_POSITION = new Pose2d(0.840,4.227, Rotation2d.fromDegrees(300));
        public static final Pose2d BLUE_3_UNDER_STAGE_POSITION = new Pose2d(4.726, 4.840, Rotation2d.fromDegrees(0));
        public static final Pose2d BLUE_3_FIRST_NOTE_POSITION = new Pose2d(7.717, 4.099, Rotation2d.fromDegrees(0));
        public static final Pose2d BLUE_3_SECOND_NOTE_POSITION =  new Pose2d(7.717, 2.425, Rotation2d.fromDegrees(0));


        // Red 1 complex auto positions  15.743
        public static final Pose2d RED_1_SCORING_POSITION = new Pose2d(15.673,6.979, Rotation2d.fromDegrees(120));
        public static final Pose2d RED_1_NEAR_AMP_POSITION = new Pose2d(14.884, 7.807, Rotation2d.fromDegrees(180));
        public static final Pose2d RED_1_FIRST_NOTE_POSITION = new Pose2d(8.796, 7.436, Rotation2d.fromDegrees(180));
        public static final Pose2d RED_1_SECOND_NOTE_POSITION = new Pose2d(8.796, 5.785, Rotation2d.fromDegrees(180));

        // Red 2 complex auto positions
        public static final Pose2d RED_2_SCORING_POSITION = new Pose2d(15.095,5.603, Rotation2d.fromDegrees(180));;
        public static final Pose2d RED_2_RIGHTMOST_NOTE_POSITION = new Pose2d(14.24, 4.088, Rotation2d.fromDegrees(180));
        public static final Pose2d RED_2_CENTER_NOTE_POSITION = new Pose2d(14.24, 5.569, Rotation2d.fromDegrees(180));
        public static final Pose2d RED_2_LEFTMOST_NOTE_POSITION = new Pose2d(14.24, 7.004, Rotation2d.fromDegrees(180));

        // Red 3 complex auto positions
        public static final Pose2d RED_3_SCORING_POSITION = new Pose2d(15.673,4.227, Rotation2d.fromDegrees(240));
        public static final Pose2d RED_3_UNDER_STAGE_POSITION = new Pose2d(11.787, 4.840, Rotation2d.fromDegrees(180));
        public static final Pose2d RED_3_FIRST_NOTE_POSITION = new Pose2d(8.796, 4.099, Rotation2d.fromDegrees(180));
        public static final Pose2d RED_3_SECOND_NOTE_POSITION = new Pose2d(8.796, 2.425, Rotation2d.fromDegrees(180));
    }

    public static final class NeoMotorConstants {
        public static final double FREE_SPEED_RPM = 5676;
    }

    public static final class FieldConstants {
        public static final Pose2d SPEAKER_POSE = new Pose2d(0, 0, new Rotation2d());
        public static final double SPEAKER_HEIGHT = 81;
    }

    public static final class IntakeConstants {
        public static final int INTAKE = 14;
    }

    public static final class ShooterConstants {
        public static final int SHOOTER = 13;
    }

    public static final class ArmConstants {
        public static final int ARM1 = 11;
        public static final int ARM2 = 12;
        public static final int PIGEON = 15;
    }
}