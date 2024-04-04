// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.List;
import java.util.function.Supplier;

import com.ctre.phoenix6.hardware.Pigeon2;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.util.WPIUtilJNI;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import frc.utils.SlewRateLimiterEX;
import frc.utils.SwerveUtils;
import frc.utils.Autonomous.AutonomousOption;
import frc.robot.Constants.ArmConstants;

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

    // Use orientation rolling for more precise turning
    private double baseRobotOrientation = 0;
    private double baseJoystickOrientation = 0;
    private boolean useOrientationTarget = false;

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

    // variables for storing past motor positions
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
    private Rotation2d driverRotationalOffset = new Rotation2d();

    // SwerveDrivePoseEstimator object to keep track of the robot's position on the field.
    private SwerveDrivePoseEstimator poseEstimator;

    // Create a supplier to make it possible for this DriveSubsystem to gain access to the cameras' estimated position.
    private Supplier<Pose2d> visionMeasurementSupplier;
    private Timer elapsedTime = new Timer();

    /**
     * Creates a new {@code DriveSubsystem} object with the specified parameters.
     * 
     * @param resetOrientation          Whether or not the IMU will be reset.
     * @param visionMeasurementSupplier A reference to a method that will provide the drivetrain with the 
     */
    public DriveSubsystem(boolean resetOrientation, Supplier<Pose2d> visionMeasurementSupplier) {

        // Reset the IMU if told to do so.
        if (resetOrientation) {
           zeroHeading();;
        }

        // Start the time so that we are able to get time stamped vision measurements.
        elapsedTime.start();

        /* 
         * Initialize up the visionMeasurementSupplier so that this DriveSubsystem is able to get
         * the camera(s) estimate of the robot's position. This helps ensure the robot is able to
         * reliable preform autonomous action, like autoscaling and auto note grabbing.
        */
        this.visionMeasurementSupplier = visionMeasurementSupplier;

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
            new Pose2d(new Translation2d(0,0), new Rotation2d(0, 0))); // Note: See how accurate this turns out to be. Change if need be.

        // Configure alternative drive mode PID controllers.
        thetaController.enableContinuousInput(0, Math.PI * 2);
    }

    @Override
    public void periodic() {
        
        // Periodically update the robot's position data to keep track of its location.
        updateRobotPose();

        Pose2d transFromRedSub = getPose().relativeTo(ArmConstants.redSubwooferPosition);
        Pose2d transFromBlueSub = getPose().relativeTo(ArmConstants.blueSubwooferPosition);
        double redDistanceAway = Math.sqrt( Math.pow(transFromRedSub.getX(), 2) + Math.pow(transFromRedSub.getY(), 2) );
        double blueDistanceAway = Math.sqrt(Math.pow(transFromBlueSub.getX(), 2) + Math.pow(transFromBlueSub.getY(), 2) );

        redDistanceAway *= 3.281;
        blueDistanceAway *= 3.281;
        double lowerDistance = Math.min(redDistanceAway, blueDistanceAway);
        double angleVal = -38.1 + 1.71*(lowerDistance) + -0.0178*Math.pow(lowerDistance, 2) + (6.19 *Math.pow(10, -5) * Math.pow(lowerDistance, 3));

        SmartDashboard.putNumber("Distance from Blue Subwoofer: ", blueDistanceAway);
        SmartDashboard.putNumber("Distance from Red Subwoofer: ", redDistanceAway);
        SmartDashboard.putNumber("AngleVal Into SmartMotion: ", angleVal);


        SmartDashboard.putString("Front Left Commanded Speed",
                Double.toString(frontLeft.getState().speedMetersPerSecond));
        SmartDashboard.putString("Front Right Commanded Speed",
                Double.toString(frontRight.getState().speedMetersPerSecond));
        SmartDashboard.putString("Back Left Commanded Speed",
                Double.toString(backLeft.getState().speedMetersPerSecond));
        SmartDashboard.putString("Back Right Commanded Speed",
                Double.toString(backRight.getState().speedMetersPerSecond));

        double currentTime = WPIUtilJNI.now() * 1e-6;
        double deltaTime = currentTime - prevTimeForEncoder;
        prevTimeForEncoder = currentTime;

        SmartDashboard.putString("Front Left Recorded Speed",
                Double.toString((frontLeft.getPosition().distanceMeters - currentFrontLeftMeters) / deltaTime));
        SmartDashboard.putString("Front Right Recorded Speed",
                Double.toString((frontRight.getPosition().distanceMeters - currentFrontRightMeters) / deltaTime));
        SmartDashboard.putString("Back Left Recorded Speed",
                Double.toString((backLeft.getPosition().distanceMeters - currentBackLeftMeters) / deltaTime));
        SmartDashboard.putString("Back Right Recorded Speed",
                Double.toString((backRight.getPosition().distanceMeters - currentBackRightMeters) / deltaTime));

        currentFrontLeftMeters = frontLeft.getPosition().distanceMeters;
        currentFrontRightMeters = frontRight.getPosition().distanceMeters;
        currentBackLeftMeters = backLeft.getPosition().distanceMeters;
        currentBackRightMeters = backRight.getPosition().distanceMeters;

         // Obtain the robot's position form the pose estimator
         Pose2d robotPose = poseEstimator.getEstimatedPosition();
        
         // Display the current estimated position of the robot
         SmartDashboard.putNumber("Robot X: ", robotPose.getX());
         SmartDashboard.putNumber("Robot Y: ", robotPose.getY());
         SmartDashboard.putNumber("Robot Heading: ", robotPose.getRotation().getDegrees());
         SmartDashboard.putNumber("IMU Heading: ", gyro.getAngle());

         // Output the current driver controller offset to check whether or not our code works.
         SmartDashboard.putNumber("Rotation Offset: ", driverRotationalOffset.getDegrees());

    }
    
    /**
     * Update the position of this {@code DriveSubsystem} to ensure accurate and up-to-date data for
     * subsequent calculations involving the robot's position.
     */
    private void updateRobotPose() {

        // Return if poseEstimator hasn't been initialize yet.
        if (poseEstimator == null) {
            return;
        }

        // Retrieve the estimated position from the robot's vision system.
        Pose2d estimatedPose2d = visionMeasurementSupplier.get();

        // Add the robot's estimated vision measurements to the pose estimator if they are not null.
        if (estimatedPose2d != null) {

            // Incorporate the robot's estimated vision measurements into this DriveSubsystem's poseEstimator.
            poseEstimator.addVisionMeasurement(
                visionMeasurementSupplier.get(), 
                elapsedTime.get());
        }

        // Update the robot's position on the field.
        poseEstimator.update(
            geRotation2d(),
            new SwerveModulePosition[] {
                frontLeft.getPosition(),
                frontRight.getPosition(),
                backLeft.getPosition(),
                backRight.getPosition()});
    }

    /**
     * sets the rotational offset for the driver
     * @param offset the offset to set to
     */
    public void setDriverRotationalOffset(Rotation2d offset) {
        driverRotationalOffset = offset;
    }

    /**
     * gets the rotational offset of the driver
     * @return the rotational offset
     */
    public Rotation2d getDriverRotationalOffset() {
        return driverRotationalOffset;
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
     * Configure the path the robot will follow and the rotational offset that will be used during teleop 
     * by using the selected autonomous command.
     * 
     * @param selectedAuto The selected autonomous that determines which path the robot will follow and 
     *                     the rotational offset during teleop.
     */
    public void setAutonomous(AutonomousOption selectedAuto) {
        setDriverRotationalOffset(selectedAuto.getTeleopRotationalOffset());
        setTrajectory(selectedAuto.getTrajectory());
        setYaw(selectedAuto.getStartingPosition().getRotation());

        // Set the robot's position to the starting location specified in the selected autonomous. We get
        // this position independent of rotation, since the position's rotation is added to the IMU's rotation,
        // which results in inaccurate readings
        resetOdometry(new Pose2d(selectedAuto.getStartingPosition().getTranslation(), gyro.getRotation2d()));
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
                if ( (double) getPose().getTranslation().getDistance(trajectory.get(waypoint).getTranslation()) < AutoConstants.TRAJECTORY_THRESHOLD) {
                        waypoint += 1;
                } else {
                        driveToPosition(trajectory.get(waypoint));
                }
        } else {
                driveToPosition(trajectory.get(waypoint - 1));
        }
    }

    /**
     * gets whether or not the trajectory is finished
     * 
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
     * @param rateLimit     Applies rate limiting to the robot
     */
    public void driveFromController(double leftX, double leftY, double rightX, double rightY, double boostMode,boolean fieldRelative, boolean rateLimit) {

        //rotates the commanded linear speed
        double oldX = leftX;
        double oldY = leftY;
        leftX = (oldX * driverRotationalOffset.getCos()) - (oldY * driverRotationalOffset.getSin());
        leftY = (oldX * driverRotationalOffset.getSin()) + (oldY * driverRotationalOffset.getCos());

        double turnJoystickOrientation = Math.atan2(rightY, rightX);
        double turnJoystickMagnitude = Math.sqrt(Math.pow(rightX, 2) + Math.pow(rightY, 2));

        if (Math.abs(rightY) > 0.8) {
            if (!useOrientationTarget) {
                useOrientationTarget = true;
                baseRobotOrientation = gyro.getRotation2d().getRadians(); // need to negate this so it matches with
                                                                          // joystick
                                                                          // orientation
                baseJoystickOrientation = turnJoystickOrientation;
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
                }

                // realigns the PID controller
                rotSpeedCommanded = thetaController.calculate(gyro.getRotation2d().getRadians(), orientationLock);

            } else {

                // reset toggle and just input speed normally
                orientationLockToggle = false;
                rotSpeedCommanded = rightX;

            }

        }

        // Applies driving to the robot
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

    /** 
     * Resets the drive encoders to currently read a position of 0. 
     */
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

    /**
     * Sets the direction that the robot is facing to the specified value.
     * 
     * @param newYaw The direction you want the robot to think it's facing
     */
    public void setYaw(Rotation2d newYaw) {
        gyro.setYaw(newYaw.getDegrees());
    }
    
    /**
     * Sets the direction that the robot is facing to the specified value.
     * 
     * @param newYaw The direction you want the robot to think it's facing
     */
    public void setYaw(double newYaw) {
        gyro.setYaw(newYaw);
    }

    /**
     * resets the heading of the robot
     */
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