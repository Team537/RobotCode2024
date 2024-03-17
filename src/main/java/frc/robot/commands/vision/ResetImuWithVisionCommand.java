package frc.robot.commands.vision;

import org.photonvision.estimation.RotTrlTransform3d;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.cameras.RobotVision;

public class ResetImuWithVisionCommand extends Command {

    // Required subsystems
    private DriveSubsystem driveSubsystem;
    private RobotVision robotVision;

    // Create a timer so that it is possible to keep track of how much time has passed in between button
    private Timer timer = new Timer();

    /* 
     * Store the current orientation of the robot before attempting to reset its heading.
     * This precaution is taken in case the attempt to reset the heading using vision fails.
     * In such a scenario, resetting the heading manually might have led to an undesirable angle,
     * and failing to reset it with vision could potentially leave the robot in a less favorable 
     * position than the initial state. By storing the robot's previous heading, we ensure a fallback 
     * option and maintain awareness of potential challenges.
     */
    private double previousHeading = -999999999;

    // Keep track of whether or not the command has finished running.
    private boolean isFinished = false;

    /**
     * Creates a new {@code ResetImuWithVisionCommand} with acsess to the specified subsystems.
     * 
     * @param driveSubsystem The robot's drivetrain
     * @param robotVision The robot's camera manager
     */
    public ResetImuWithVisionCommand(DriveSubsystem driveSubsystem, RobotVision robotVision) {

        // Gain acsess to the robot's subsystems so that we are able to interact with them.
        this.driveSubsystem = driveSubsystem;
        this.robotVision = robotVision;

        // Start the timer
        timer.start();
    }

    @Override
    public void initialize() {

        // Make it possible to run the comand again. If we don't set isFinished to false every tine
        // this command is run, then the command will only sucsessfully run once, since once it completes 
        isFinished = false;
        
        // Determine whether or not the driver wants to use vision to reset the robot's gyro. 
        // This is done by checking if we pressed the back button within the last 750 milliseconds.
        if (timer.get() > 0.75 || previousHeading == -999999999) {
            System.out.println("------------------------------------------------");
            resetGyro(); // We don't want to use vision to rset the robot's gyro.
        }

        // Reset the timer so that we can keep track of the time that has passed sicne this command
        // started running. This helps us determine when we want to reset the robot's heading with
        // april tags as well as when we want to 
        timer.reset();
    }
    
    /**
     * Runs periodically while the command is running. This section of the code will only run 
     * if we are attempting to reset the IMU using the 
     */
    @Override
    public void execute() {
        resetGyroWithVision();
    }

    /**
     * Attempt to reset the robot's heading using the various april tags located around the field. If 
     * the robot is unable accomplish this in under 250 milliseconds, then in
     */
    private void resetGyroWithVision() {
        
        // Try and get an estimate of the robot's position. If we are unable to get a good estimate,
        // then wait until we are able to get a good estimate of the robot's position. After that, then 
        // see if it has taken more than 250 milliseconds have passed since we started trying to reset the IMU
        // with vision.
        Pose2d estimatedRobotPose = robotVision.estimateRobotPose();
        if (estimatedRobotPose == null) {

            // If more then 250 milliseconds have passed then restore the robot's previous orientation and 
            // print out an error.
            if (timer.get() > .25) {

                // Restore the robot's original orientation.
                restoreOrientation();

                // Inform the user of the failure.
                System.err.println("Unable to reset robot's IMU with vision.");

                // Stop running the command
                isFinished = true;
            }

            return; // Robot can't see any AprilTags.
        }

        // Get the direction that we think the robot is facing from the estimated position.
        Rotation2d estimatedRobotYaw = estimatedRobotPose.getRotation();

        // Correct the robot's IMU so that the direction the robot thinks that is is facing forwards when
        // it's facing the center of the field. 
        driveSubsystem.setYaw(estimatedRobotYaw);

        // Tell the robot that the command has finished running.
        isFinished = true;
    }
    
    /**
     * Resets the robot heading without using vision.
     */
    private void resetGyro() {

        // Store the robot's current heading in case we need to revert back to it later.
        previousHeading = driveSubsystem.getHeading();
        
        // Reset the robot's gyro.
        driveSubsystem.setYaw(driveSubsystem.getDriverRotationalOffset());

        // Tell the robot that the command has finished running.
        isFinished = true;
    }

    /**
     * Reset the IMU's zero position back to what it was prior to this command being run.
     */
    private void restoreOrientation() {

        /*
         * -----------------------------------------------------------------------------------------------
         * NOTE: Robot's current heading is an offset from it's previous heading, since it's zero position 
         * is the angle it previously thought to be the value stored in previousHeading.
         * -----------------------------------------------------------------------------------------------
         * 
         * Add the robot's previous heading and the rotational offset minus our current heading (How much 
         * we turrned since we started trying to reset the IMU with vision) together. Doing this is necessary, 
         * as the robot may have rotated since when we last stored it's rotation. 
         */
        double updatedPreviousHeading = previousHeading + (driveSubsystem.getHeading() 
                - driveSubsystem.getDriverRotationalOffset().getRadians());

        // Revert the robot's imu's zero position back to what it was previously.
        driveSubsystem.setYaw(Rotation2d.fromRadians(updatedPreviousHeading));
    }

    @Override
    public void end(boolean interrupted) {}

    @Override
    public boolean isFinished() {
        return isFinished;
    }
}