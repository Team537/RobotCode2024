// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.List;
import java.util.function.Supplier;

import com.ctre.phoenix6.hardware.Pigeon2;

import edu.wpi.first.math.controller.HolonomicDriveController;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.util.WPIUtilJNI;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import frc.utils.SlewRateLimiterEX;
import frc.utils.SwerveUtils;

public class DriveSubsystem extends SubsystemBase {

    // Create MAXSwerveModules
    private final MAXSwerveModule frontLeft = new MAXSwerveModule(
            DriveConstants.FRONT_LEFT_DRIVE_CAN_ID,
            DriveConstants.FRONT_LEFT_TURNING_CAN_ID,
            DriveConstants.FRONT_LEFT_CHASSIS_ANGULAR_OFFSET);

    private final MAXSwerveModule frontRight = new MAXSwerveModule(
            DriveConstants.FRONT_RIGHT_DRIVE_CAN_ID,
            DriveConstants.FRONT_RIGHT_TURNING_CAN_ID,
            DriveConstants.FRONT_RIGHT_CHASSIS_ANGULAR_OFFSET);

    private final MAXSwerveModule backLeft = new MAXSwerveModule(
            DriveConstants.BACK_LEFT_DRIVE_CAN_ID,
            DriveConstants.BACK_LEFT_TURNING_CAN_ID,
            DriveConstants.BACK_LEFT_CHASSIS_ANGULAR_OFFSET);

    private final MAXSwerveModule backRight = new MAXSwerveModule(
            DriveConstants.BACK_RIGHT_DRIVE_CAN_ID,
            DriveConstants.BACK_RIGHT_TURNING_CAN_ID,
            DriveConstants.BACK_RIGHT_CHASSIS_ANGULAR_OFFSET);

    // The gyro sensor
    private final Pigeon2 gyro = new Pigeon2(42);
    public static double driveYaw = 0;

    private boolean orientationLockToggle = false;
    private double orientationLock = 0;
    private PIDController orientationLockController = new PIDController(DriveConstants.ORIENTATION_LOCK_KP,
            DriveConstants.ORIENTATION_LOCK_KI, DriveConstants.ORIENTATION_LOCK_KD);

    // Use orientation rolling for more precise turning
    private double baseRobotOrientation = 0;
    private double baseJoystickOrientation = 0;
    private boolean useOrientationTarget = false;
    private PIDController targetOrientationController = new PIDController(DriveConstants.ORIENTATION_LOCK_KP,
            DriveConstants.ORIENTATION_LOCK_KI, DriveConstants.ORIENTATION_LOCK_KD);

    // Slew rate filter variables for controlling lateral acceleration
    private double currentRotation = 0.0;
    private double currentTranslationDir = 0.0;
    private double currentTranslationMag = 0.0;
    private double currentMaxSpeed = DriveConstants.MAX_SPEED_METERS_PER_SECOND;

    private SlewRateLimiterEX magLimiter = new SlewRateLimiterEX(DriveConstants.MAGNITUDE_POSITIVE_SLEW_RATE,
            DriveConstants.MAGNITUDE_NEGATIVE_SLEW_RATE);
    private SlewRateLimiter rotLimiter = new SlewRateLimiter(DriveConstants.ROTATIONAL_SLEW_RATE);
    private double prevTime = WPIUtilJNI.now() * 1e-6;

    private ProfiledPIDController thetaController = new ProfiledPIDController(
            AutoConstants.THETA_CONTROLLER_KP, 0, 0, AutoConstants.kThetaControllerConstraints);

    private HolonomicDriveController driveController = new HolonomicDriveController(
            new PIDController(AutoConstants.X_CONTROLLER_KP, 0, 0),
            new PIDController(AutoConstants.Y_CONTROLLER_KP, 0, 0), thetaController);

    // Create config for trajectory
    private TrajectoryConfig config = new TrajectoryConfig(
            AutoConstants.MAX_SPEED_METERS_PER_SECOND,
            AutoConstants.MAX_ACCELERATION_METERS_PER_SECOND_SQUARED)
            // Add kinematics to ensure max speed is actually obeyed
            .setKinematics(DriveConstants.DRIVE_KINEMATICS);
    private Trajectory exampleTrajectory = TrajectoryGenerator.generateTrajectory(
            // Start at the origin facing the +X direction
            new Pose2d(0, 0, new Rotation2d(0)),
            // Pass through these two interior waypoints, making an 's' curve path
            List.of(new Translation2d(1, 1), new Translation2d(2, -1)),
            // End 3 meters straight ahead of where we started, facing forward
            new Pose2d(3, 0, new Rotation2d(0)),
            config);

    // SwerveDrivePoseEstimator object to keep track of the robot's position on the field.
    private SwerveDrivePoseEstimator poseEstimator;

    // Create a supplier to make it possible for this DriveSubsystem to gain acsess to the cameras' estimated position.
    // private Supplier<Pose2d> visionMeasurementSupplier;

    /**
     * Creates a new {@code DriveSubsystem} object with the specified paramaters.
     * 
     * @param resetOrientation          Whether or not the IMU will be reset.
     * @param visionMeasurementSupplier A refference to a method that will provide the drivetrain with the 
     */
    public DriveSubsystem(boolean resetOrientation) {

        // Reset the IMU if told to do so.
        if (resetOrientation) {
            zeroHeading();
        }

        /* 
         * Initialze up the visionMeasurementSupplier so that this DriveSubsystem is able to get
         * the camera(s) estimate of the robot's position. This helps ensure the robot is able to
         * reliable prefrom autonomous action, like autoscoring and auto note grabbing.
        */
        // this.visionMeasurementSupplier = visionMeasurementSupplier;

         // Setup the robot's PoseEstimator so that we are able to get the robot's position on the 
        // field at any given time.
        poseEstimator = new SwerveDrivePoseEstimator(
            DriveConstants.DRIVE_KINEMATICS, 
            gyro.getRotation2d(), 
            new SwerveModulePosition[] {
                frontLeft.getPosition(),
                frontRight.getPosition(),
                backLeft.getPosition(),
                backRight.getPosition()},
            new Pose2d(new Translation2d(0, 0), new Rotation2d(0, 0))); // Note: See how accurate this turns out to be. Change if need be.

        // Configure alternative drive mode PID controllers.
        orientationLockController.enableContinuousInput(0, Math.PI * 2);
        targetOrientationController.enableContinuousInput(0, Math.PI * 2);
        thetaController.enableContinuousInput(-Math.PI, Math.PI);
    }

    @Override
    public void periodic() {

        // Periodically update the robot's position data to keep track of its location.
        updateRobotPose();
        driveYaw = gyro.getYaw().getValue();
    }
    
    /**
     * Update the position of this {@code DriveSubsystem} to ensure accurate and up-to-date data for
     * subsequent calculations involving the robot's position.
     */
    private void updateRobotPose() {

        // Return if poseEstimator hasn't been intialize yet.
        if (poseEstimator == null) {
            return;
        }

        // Retrieve the estimated position from the robot's vision system.
        // Pose2d estimatedPose2d = visionMeasurementSupplier.get();

        // Add the robot's estimated vision measurments to the pose estimator if they are not null.
        // if (estimatedPose2d != null) {

        //     // Incorporate the robot's estimated vision measurements into this DriveSubsystem's poseEstimator.
        //     poseEstimator.addVisionMeasurement(
        //         visionMeasurementSupplier.get(), 
        //         Timer.getFPGATimestamp());
        // }

        // Update t `he robot's position on the field.
        poseEstimator.update(
            geRotation2d(),
            new SwerveModulePosition[] {
                frontLeft.getPosition(),
                frontRight.getPosition(),
                backLeft.getPosition(),
                backRight.getPosition()});
    }

    // NOTE: Please change the name of this method to make it's purpose more clear
    /**
     * basic layout for targeting a position
     * 
     * @param targetPose robot for pose to target
     */
    public void position(Pose2d targetPose) {

        setModuleStates(DriveConstants.DRIVE_KINEMATICS
                .toSwerveModuleStates(driveController.calculate(getPose(), targetPose,
                 0, 
                 targetPose.getRotation())));

        // System.out.println(getPose());
    }

    /**
     * Method to drive the robot using joystick info.
     *
     * @param xSpeed        Speed of the robot in the x direction (forward).
     * @param ySpeed        Speed of the robot in the y direction (sideways).
     * @param rot           Angular rate of the robot.
     * @param fieldRelative Whether the provided x and y speeds are relative to the
     *                      field.
     * @param rateLimit     Whether to enable rate limiting for smoother control.
     */
    public void drive(double xSpeed, double ySpeed, double rot, double orientationRoll, double boostMode,
            boolean fieldRelative, boolean rateLimit) {

        double turnJoystickMagnitude = Math.sqrt(Math.pow(rot, 2) + Math.pow(orientationRoll, 2));
        double turnJoystickOrientation = Math.atan2(orientationRoll, rot);

        if (Math.abs(orientationRoll) > 0.8) {
            if (!useOrientationTarget) {
                useOrientationTarget = true;
                baseRobotOrientation = gyro.getRotation2d().getRadians(); // need to negate this so it matches with
                                                                          // joystick orientation
                baseJoystickOrientation = turnJoystickOrientation;
                targetOrientationController.reset();
            }
        }

        if (turnJoystickMagnitude < 1e-4) {
            useOrientationTarget = false;
        }

        // Curve the inputs for easier more precise driving (don't curve rotation for
        // orientation rolling)
        double linearCurveMultiplier = Math.pow(Math.sqrt(Math.pow(xSpeed, 2) + Math.pow(ySpeed, 2)),
                Constants.OIConstants.INPUT_CURVE_POWER - 1);
        double angularCurveMultiplier = Math.pow(Math.abs(rot), Constants.OIConstants.INPUT_CURVE_POWER - 1);
        xSpeed *= linearCurveMultiplier;
        ySpeed *= linearCurveMultiplier;
        if (useOrientationTarget) {
            rot *= angularCurveMultiplier;
        }

        double xSpeedCommanded;
        double ySpeedCommanded;
        double rotSpeedCommanded;

        // target orientation
        if (useOrientationTarget) {
            rotSpeedCommanded = targetOrientationController.calculate(gyro.getRotation2d().getRadians(),
                    -turnJoystickOrientation + baseJoystickOrientation + baseRobotOrientation);
        } else {
            rotSpeedCommanded = rot;
        }

        double newMaxSpeed = DriveConstants.MAX_SPEED_METERS_PER_SECOND + boostMode
                * (DriveConstants.BOOST_MODE_MAX_SPEED_METERS_PER_SECOND - DriveConstants.MAX_SPEED_METERS_PER_SECOND);

        // greater than to correct for precision errors
        boolean useBoostMode = boostMode > 1e-4;

        // only scales if the rate limiter is being used
        if (rateLimit) {
            magLimiter.scalePrev(currentMaxSpeed / newMaxSpeed);
        }

        currentMaxSpeed = newMaxSpeed;

        if (rateLimit) {
            // Convert XY to polar for rate limiting
            double inputTranslationMag = Math.sqrt(Math.pow(xSpeed, 2) + Math.pow(ySpeed, 2));
            double inputTranslationDir = (inputTranslationMag > 1e-4) ? Math.atan2(ySpeed, xSpeed)
                    : currentTranslationDir; // don't change direction if the magnitude is 0
            // double inputTranslationDir = Math.atan2(ySpeed, xSpeed);

            // Calculate the direction slew rate based on an estimate of the lateral
            // acceleration
            double directionSlewRate;
            if (currentTranslationMag != 0.0) {
                directionSlewRate = Math.abs(DriveConstants.DIRECTION_SLEW_RATE / currentTranslationMag);
            } else {
                directionSlewRate = 500.0; // some high number that means the slew rate is effectively instantaneous
            }

            double currentTime = WPIUtilJNI.now() * 1e-6;
            double elapsedTime = currentTime - prevTime;
            double angleDif = SwerveUtils.AngleDifference(inputTranslationDir, currentTranslationDir);

            boolean enableGrip = angleDif > 0.1 * Math.PI && currentTranslationMag > 1e-4 && inputTranslationMag > 1e-4;

            // quickly accelerate if changing directions (don't use if boost mode is not
            // activated)
            if (!useBoostMode) {
                if (enableGrip) {
                    magLimiter.setRate(Constants.DriveConstants.GRIP_MAGNITUDE_POSITIVE_SLEW_RATE,
                            Constants.DriveConstants.GRIP_MAGNITUDE_NEGATIVE_SLEW_RATE);
                    directionSlewRate = Math.abs(DriveConstants.GRIP_DIRECTION_SLEW_RATE / currentTranslationMag);
                } else {
                    magLimiter.setRate(Constants.DriveConstants.MAGNITUDE_POSITIVE_SLEW_RATE,
                            Constants.DriveConstants.MAGNITUDE_NEGATIVE_SLEW_RATE);
                }
            } else {
                if (enableGrip) {
                    directionSlewRate = Math
                            .abs(DriveConstants.GRIP_BOOST_MODE_DIRECTION_SLEW_RATE / currentTranslationMag);
                } else {
                    directionSlewRate = Math.abs(DriveConstants.BOOST_MODE_DIRECTION_SLEW_RATE / currentTranslationMag);
                }
                magLimiter.setRate(DriveConstants.BOOST_MODE_MAGNITUDE_POSITIVE_SLEW_RATE,
                        DriveConstants.BOOST_MODE_MAGNITUDE_NEGATIVE_SLEW_RATE);
            }

            if (angleDif < 0.45 * Math.PI) {
                currentTranslationDir = SwerveUtils.StepTowardsCircular(currentTranslationDir, inputTranslationDir,
                        directionSlewRate * elapsedTime);
                currentTranslationMag = magLimiter.calculate(inputTranslationMag);
            } else if (angleDif > 0.85 * Math.PI) {
                if (currentTranslationMag > 1e-4) { // some small number to avoid floating-point errors with equality
                                                    // checking
                    // keep currentTranslationDir unchanged
                    currentTranslationMag = magLimiter.calculate(0);
                } else {
                    currentTranslationDir = SwerveUtils.WrapAngle(currentTranslationDir + Math.PI);
                    currentTranslationMag = magLimiter.calculate(inputTranslationMag);
                }
            } else {
                currentTranslationDir = SwerveUtils.StepTowardsCircular(currentTranslationDir, inputTranslationDir,
                        directionSlewRate * elapsedTime);
                currentTranslationMag = magLimiter.calculate(0);
            }
            prevTime = currentTime;

            xSpeedCommanded = currentTranslationMag * Math.cos(currentTranslationDir);
            ySpeedCommanded = currentTranslationMag * Math.sin(currentTranslationDir);
            currentRotation = rotLimiter.calculate(rotSpeedCommanded);

            // if orientation is zero, begin storing the orientation lock
            if (Math.abs(currentRotation) < 1e-4 && !useOrientationTarget) {

                // only activate on the rising edge
                if (!orientationLockToggle) {

                    orientationLockToggle = true;
                    orientationLock = gyro.getRotation2d().getRadians();

                    // resets integral value on PID controller
                    orientationLockController.reset();

                }

                // realigns the PID controller
                currentRotation = orientationLockController.calculate(gyro.getRotation2d().getRadians(),
                        orientationLock);
            } else {
                orientationLockToggle = false;
            }

        } else {
            xSpeedCommanded = xSpeed;
            ySpeedCommanded = ySpeed;
            currentRotation = rotSpeedCommanded;
        }

        // Convert the commanded speeds into the correct units for the drivetrain
        double xSpeedDelivered = xSpeedCommanded * currentMaxSpeed;
        double ySpeedDelivered = ySpeedCommanded * currentMaxSpeed;
        double rotDelivered = currentRotation * DriveConstants.MAX_ANGULAR_SPEED;

        var swerveModuleStates = DriveConstants.DRIVE_KINEMATICS.toSwerveModuleStates(
                fieldRelative
                        ? ChassisSpeeds.fromFieldRelativeSpeeds(xSpeedDelivered, ySpeedDelivered, rotDelivered,
                                gyro.getRotation2d())
                        : new ChassisSpeeds(xSpeedDelivered, ySpeedDelivered, rotDelivered));
        SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates,
                DriveConstants.BOOST_MODE_MAX_SPEED_METERS_PER_SECOND);
        frontLeft.setDesiredState(swerveModuleStates[0]);
        frontRight.setDesiredState(swerveModuleStates[1]);
        backLeft.setDesiredState(swerveModuleStates[2]);
        backRight.setDesiredState(swerveModuleStates[3]);

        // System.out.println("gyro: " + gyro.getRotation2d().getDegrees());
        // System.out.println(swerveModuleStates[0].angle);
    }

    /**
     * Sets the wheels into an X formation to prevent movement.
     */
    public void setX() {
        frontLeft.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(45)));
        frontRight.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(-45)));
        backLeft.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(-45)));
        backRight.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(45)));
    }

    /**
     * Sets the swerve ModuleStates.
     *
     * @param desiredStates The desired SwerveModule states.
     */
    public void setModuleStates(SwerveModuleState[] desiredStates) {
        SwerveDriveKinematics.desaturateWheelSpeeds(
                desiredStates, DriveConstants.BOOST_MODE_MAX_SPEED_METERS_PER_SECOND);
        frontLeft.setDesiredState(desiredStates[0]);
        frontRight.setDesiredState(desiredStates[1]);
        backLeft.setDesiredState(desiredStates[2]);
        backRight.setDesiredState(desiredStates[3]);
    }

    /** Resets the drive encoders to currently read a position of 0. */
    public void resetEncoders() {
        frontLeft.resetEncoders();
        backLeft.resetEncoders();
        frontRight.resetEncoders();
        backRight.resetEncoders();
    }

    /**
     * Resets the odometry to the specified pose.
     *
     * @param pose The new position of the robot.
     */
    public void resetOdometry(Pose2d pose) {
        poseEstimator.resetPosition(
            gyro.getRotation2d(),
            new SwerveModulePosition[] {
                frontLeft.getPosition(),
                frontRight.getPosition(),
                backLeft.getPosition(),
                backRight.getPosition()},
            pose);
    }

    /** Zeroes the heading of the robot. */
    public void zeroHeading() {
        gyro.reset();
    }

    /**
     * Returns the robot's position on the field as a {@code Pose2d}.
     *
     * @return The robot's position on the field as a {@code Pose2d}.
     */
    public Pose2d getPose() {
        return poseEstimator.getEstimatedPosition();
    }

    /**
     * Returns the robot's rotation, as a {@code Rotation2d}.
     * 
     * @return The robot's rotation, as a {@code Rotation2d}.
     */
    public Rotation2d geRotation2d() {
        return gyro.getRotation2d();
    }

    /**
     * Returns the heading of the robot.
     *
     * @return the robot's heading in radians, from -pi to pi.
     */
    public double getHeading() {
        return gyro.getRotation2d().getRadians();
    }

    /**
     * Returns the turn rate of the robot.
     *
     * @return The turn rate of the robot, in degrees per second
     */
    public double getTurnRate() {
        return gyro.getRate();
    }
}
