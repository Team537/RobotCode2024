// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.List;

import com.ctre.phoenix6.hardware.Pigeon2;

import edu.wpi.first.math.controller.HolonomicDriveController;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.util.WPIUtilJNI;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
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

    private boolean orientationLockToggle = false;
    private boolean useOrientationLock = false;
    private double orientationLock = 0;
    private PIDController orientationLockController = new PIDController(DriveConstants.ORIENTATION_LOCK_KP,
            DriveConstants.ORIENTATION_LOCK_KI, DriveConstants.ORIENTATION_LOCK_KD);

    // use orientation rolling for more precise turning
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

    // other "prevTime" used for displaying motor outputs
    private double prevTimeForEncoder = WPIUtilJNI.now() * 1e-6;

    // variables for storing past motor postions
    private double currentFrontLeftMeters = 0;
    private double currentFrontRightMeters = 0;
    private double currentBackLeftMeters = 0;
    private double currentBackRightMeters = 0;

    PIDController thetaController = new PIDController(
            AutoConstants.THETA_CONTROLLER_KP, 0, AutoConstants.THETA_CONTROLLER_KD);

    PIDController xController = new PIDController(AutoConstants.X_CONTROLLER_KP,0,0);
    PIDController yController = new PIDController(AutoConstants.Y_CONTROLLER_KP,0,0);

    List<Pose2d> trajectory = List.of();
    private int waypoint = 0;

    //the rotational offset from where the driver is facing to where the robot considers forward. This is so we can standardize position data for both alliances
    private double driverRotationalOffset = 0;

    // Odometry class for tracking robot pose
    SwerveDriveOdometry odometry = new SwerveDriveOdometry(
            DriveConstants.DRIVE_KINEMATICS,
            gyro.getRotation2d(),
            new SwerveModulePosition[] {
                    frontLeft.getPosition(),
                    frontRight.getPosition(),
                    backLeft.getPosition(),
                    backRight.getPosition()
            });

    /** Creates a new DriveSubsystem. */
    public DriveSubsystem() {
        zeroHeading();
        orientationLockController.enableContinuousInput(0, Math.PI * 2);
        targetOrientationController.enableContinuousInput(0, Math.PI * 2);
        thetaController.enableContinuousInput(0, Math.PI * 2);
    }

    @Override
    public void periodic() {
        // Update the odometry in the periodic block
        odometry.update(
                gyro.getRotation2d(),
                new SwerveModulePosition[] {
                        frontLeft.getPosition(),
                        frontRight.getPosition(),
                        backLeft.getPosition(),
                        backRight.getPosition()
                });

        SmartDashboard.putString("heading", Double.toString(gyro.getRotation2d().getRadians()));
        SmartDashboard.putString("x", Double.toString(getPose().getX()));
        SmartDashboard.putString("y", Double.toString(getPose().getY()));

        SmartDashboard.putString("Front Left Commanded Speed",
                Double.toString(frontLeft.getState().speedMetersPerSecond));
        SmartDashboard.putString("Front Right Commanded Speed",
                Double.toString(frontRight.getState().speedMetersPerSecond));
        SmartDashboard.putString("Back Left Commanded Speed",
                Double.toString(backLeft.getState().speedMetersPerSecond));
        SmartDashboard.putString("Back Right Commanded Speed",
                Double.toString(backRight.getState().speedMetersPerSecond));

        double currentTime = WPIUtilJNI.now() * 1e-6;
        double elapsedTime = currentTime - prevTimeForEncoder;
        prevTimeForEncoder = currentTime;

        SmartDashboard.putString("Front Left Recorded Speed",
                Double.toString((frontLeft.getPosition().distanceMeters - currentFrontLeftMeters) / elapsedTime));
        SmartDashboard.putString("Front Right Recorded Speed",
                Double.toString((frontRight.getPosition().distanceMeters - currentFrontRightMeters) / elapsedTime));
        SmartDashboard.putString("Back Left Recorded Speed",
                Double.toString((backLeft.getPosition().distanceMeters - currentBackLeftMeters) / elapsedTime));
        SmartDashboard.putString("Back Right Recorded Speed",
                Double.toString((backRight.getPosition().distanceMeters - currentBackRightMeters) / elapsedTime));

        currentFrontLeftMeters = frontLeft.getPosition().distanceMeters;
        currentFrontRightMeters = frontRight.getPosition().distanceMeters;
        currentBackLeftMeters = backLeft.getPosition().distanceMeters;
        currentBackRightMeters = backRight.getPosition().distanceMeters;

    }

    /**
     * sets the rotational offset for the driver
     * @param offset the offset to set to
     */
    public void setDriverRotationalOffset(double offset) {
        driverRotationalOffset = offset;
    }

    /**
     * Resets the odometry to the specified pose.
     *
     * @param pose The pose to which to set the odometry.
     */
    public void resetOdometry(Pose2d pose) {
        odometry.resetPosition(
                gyro.getRotation2d(),
                new SwerveModulePosition[] {
                        frontLeft.getPosition(),
                        frontRight.getPosition(),
                        backLeft.getPosition(),
                        backRight.getPosition()
                },
                pose);
    }

    /**
     * basic layout for targeting a position
     * 
     * @param targetPose robot for pose to target
     */
    public void driveToPosition(Pose2d targetPose) {

        double thetaSpeed = thetaController.calculate(getPose().getRotation().getRadians(),targetPose.getRotation().getRadians());
        double xSpeed = xController.calculate(getPose().getX(),targetPose.getX());
        double ySpeed = yController.calculate(getPose().getY(),targetPose.getY());

        drive(xSpeed,ySpeed,thetaSpeed,0,true,true);

    }

    /**
     * resets the trajectory to follow it from the beginning
     */
    public void resetTrajectory() {
        waypoint = 0;
    }

    /**
     * sets the trajectory to follow
     * @param trajectory the trajectory to follow
     */
    public void setTrajectory(List<Pose2d> trajectory) {
        this.trajectory = trajectory;
        resetTrajectory();
    }

    /**
     * follows a trajectory
     */
    public void followTrajectory() {
        if (waypoint < trajectory.size()) {
                if (getPose().getTranslation().getDistance(trajectory.get(waypoint).getTranslation()) < AutoConstants.TRAJECOTRY_THRESHOLD) {
                        waypoint += 1;
                } else {
                        driveToPosition(trajectory.get(waypoint));
                }
        } else {
                driveToPosition(trajectory.get(waypoint - 1));
        }
    }

    /**
     * gets whether or not the trajcetory is finished
     * @return whether or not the trajectory is finished
     */
    public boolean isTrajectoryFinished() {
        if (waypoint < trajectory.size()) {
                return false;
        } else {
                return true;
        }
    }



    /**
     * Returns the currently-estimated pose of the robot.
     *
     * @return The pose.
     */
    public Pose2d getPose() {
        return odometry.getPoseMeters();
    }

    /**
     * drives robot from inputs
     * 
     * @param leftX         the linear input for the x direction, usually the left
     *                      joystick x
     * @param leftY         the linear input for the y direction, usually the left
     *                      joystick y
     * @param rightX        the rotation input for the robot, usually the right
     *                      joystick x, also used for orientation targeting
     * @param rightY        used for rotation targeting along with rightX, usually
     *                      the right joystick y
     * @param boostMode     controls the robot's max speed
     * @param fieldRelative determines whether the robot is field centric
     * @param rateLimit     applys rate limiting to the robot
     */
    public void driveFromController(double leftX, double leftY, double rightX, double rightY, double boostMode,boolean fieldRelative, boolean rateLimit) {

        //rotates the commanded linear speed
        double oldX = leftX;
        double oldY = leftY;
        leftX = oldX * Math.cos(driverRotationalOffset) - oldY * Math.sin(driverRotationalOffset);
        leftX = oldY * Math.sin(driverRotationalOffset) + oldY * Math.cos(driverRotationalOffset);

        double turnJoystickOrientation = Math.atan2(rightY, rightX);
        double turnJoystickMagnitude = Math.sqrt(Math.pow(rightX, 2) + Math.pow(rightY, 2));

        if (Math.abs(rightY) > 0.8) {
            if (!useOrientationTarget) {
                useOrientationTarget = true;
                baseRobotOrientation = gyro.getRotation2d().getRadians(); // need to negate this so it matches with
                                                                          // joystick
                                                                          // orientation
                baseJoystickOrientation = turnJoystickOrientation;
                targetOrientationController.reset();
            }
        }

        if (turnJoystickMagnitude < 0.3) {
            useOrientationTarget = false;
        }

        // Curve the inputs for easier more precise driving (don't curve rotation for
        // orientation rolling)
        double linearCurveMultiplier = Math.pow(Math.sqrt(Math.pow(leftX, 2) + Math.pow(leftY, 2)),
                Constants.OIConstants.INPUT_CURVE_POWER - 1);
        double angularCurveMultiplier = Math.pow(Math.abs(rightX), Constants.OIConstants.INPUT_CURVE_POWER - 1);
        leftX *= linearCurveMultiplier;
        leftY *= linearCurveMultiplier;
        if (useOrientationTarget) {
            rightX *= angularCurveMultiplier;
        }

        double rotSpeedCommanded;

        // target orientation
        if (useOrientationTarget) {
            rotSpeedCommanded = thetaController.calculate(gyro.getRotation2d().getRadians(),
                    -turnJoystickOrientation + baseJoystickOrientation + baseRobotOrientation);
        } else {

        
            if (Math.abs(rightX) < 1e-4 && currentRotation == 0) {
                 useOrientationLock = true;
            }

            if (useOrientationLock) {
                if (!(Math.abs(rightX) < 1e-4)) {
                        useOrientationLock = false;
                }
            }
            // if orientation speed is zero, begin storing the orientation lock
            if (useOrientationLock) {

                // only activate on the rising edge
                if (!orientationLockToggle) {

                    orientationLockToggle = true;

                    orientationLockToggle = true;
                    orientationLock = gyro.getRotation2d().getRadians();

                    // resets integral value on PID controller
                    orientationLockController.reset();

                }

                // realigns the PID controller
                rotSpeedCommanded = thetaController.calculate(gyro.getRotation2d().getRadians(), orientationLock);

            } else {

                // reset toggle and just input speed normally
                orientationLockToggle = false;
                rotSpeedCommanded = rightX;

            }

        }

        // applys driving to the robot
        drive(leftX, leftY, rotSpeedCommanded, boostMode, true, true);

    }

    /**
     * drives the robot
     * 
     * @param xSpeed        linear speed in the x direction
     * @param ySpeed        linear speed in the y direction
     * @param rot           rotation speed
     * @param boostMode     max speed and acceleration controls
     * @param rateLimit     limits the rate of the robot
     * @param fieldRelative enables field centricity
     */
    public void drive(double xSpeed, double ySpeed, double rot, double boostMode, boolean rateLimit,
            boolean fieldRelative) {

        double xSpeedCommanded;
        double ySpeedCommanded;

        if (Math.abs(rot) > 1) {
                rot /= Math.abs(rot);
        }

        if (Math.abs(xSpeed) > 1) {
                xSpeed /= Math.abs(xSpeed);
        }

        if (Math.abs(ySpeed) > 1) {
                ySpeed /= Math.abs(ySpeed);
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
            currentRotation = rotLimiter.calculate(rot);

        } else {
            xSpeedCommanded = xSpeed;
            ySpeedCommanded = ySpeed;
            currentRotation = rot;
        }

        // Convert the commanded speeds into the correct units for the drivetrain
        double xSpeedDelivered = xSpeedCommanded * currentMaxSpeed;
        double ySpeedDelivered = ySpeedCommanded * currentMaxSpeed;
        double rotDelivered = currentRotation * DriveConstants.MAX_ANGULAR_SPEED;

        if (!(Double.isFinite(xSpeedDelivered) && Double.isFinite(ySpeedDelivered) && Double.isFinite(rotDelivered))) {
                xSpeedDelivered = 0;
                ySpeedDelivered = 0;
                rotDelivered = 0;
        }

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

    /** Zeroes the heading of the robot. */
    public void zeroHeading() {
        gyro.reset();
    }

    /**
     * Returns the heading of the robot.
     *
     * @return the robot's heading in degrees, from -180 to 180
     */
    public double getHeading() {
        return gyro.getRotation2d().getDegrees();
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
