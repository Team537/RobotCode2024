package frc.robot.commands.vision;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.cameras.RobotVision;

public class ResetImuWithVisionCommand extends Command {

    // Subsystems
    private DriveSubsystem driveSubsystem;
    private RobotVision robotVision;

    // Prevent the program from running this command indefinitly, since scoring points is
    // likely to be more worthwhile.
    private final int MAX_FAILED_EStIMATION_COUNT = 250; // Itterations of the command
    private int failedEstimteionCount = 0;

    // Flags
    private boolean isFinished = false;


    public ResetImuWithVisionCommand(DriveSubsystem driveSubsystem, RobotVision robotVision) {

        // Gain acsess to the robot's subsystems
        this.driveSubsystem = driveSubsystem;
        this.robotVision = robotVision;
    }

    @Override
    public void initialize() {
        isFinished = false;
    }

    @Override
    public void execute() {

        // Try and get an estimante of the robot's position. If we are unable to get a good estimante,
        // then wait until we are able to get a good estimante of the robot's position. Additionally, 
        // increase failedEstimteionCount by 1 so that we can keep track of how many times we faled to
        // reset the IMU 
        Pose2d estimatedRobotPose = robotVision.estimateRobotPose();
        if (estimatedRobotPose == null) {
            return;
        }

        // Get the direction that we think the robot is facing from the estimated position.
        double estimatedRobotYaw = estimatedRobotPose.getRotation().getDegrees();

        // Correct the robot's IMU so that the direction the robot thinks that is is facing forwards when
        // it's facing the center of the field. 
        driveSubsystem.setYaw(estimatedRobotYaw);

        // Tell the robot that the command has finished runnng.
        isFinished = true;
    }

    @Override
    public void end(boolean interrupted) {}

    @Override
    public boolean isFinished() {
        return isFinished;
    }
}
