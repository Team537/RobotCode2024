package frc.robot.commands;

import java.util.List;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DriveSubsystem;

/**
 * This command tells the robot to drive to a list of positions on the field.
 */
public class FollowTrajectoryCommand extends Command {
    
    private DriveSubsystem driveSubsystem;
    private List<Pose2d> trajectory;
    private boolean isFinished = false;

    /**
     * Setup everything so that this command can access ad interact with the robot's drivetrain.
     * 
     * @param driveSubsystem The robot's drivetrain.
     * @param trajectory The path the robot will drive along.
     */
    public FollowTrajectoryCommand(DriveSubsystem driveSubsystem, List<Pose2d> trajectory) {

        // Gain access to the robot's drivetrain.
        this.driveSubsystem = driveSubsystem;

        // Setup the path that the robot will follow.
        this.trajectory = trajectory;
        
        // Make sure that 2 commands that involve driving don't happen at the same time.
        addRequirements(driveSubsystem);
    }

    /**
     * Tell the robot what trajectory it will follow and then tell it to drive along that path.
     */
    @Override
    public void initialize() {
        isFinished = false;
        driveSubsystem.setTrajectory(this.trajectory);
    }

    /**
     * Follow the trajectory while the command is active.
     */
    public void execute() {
        driveSubsystem.followTrajectory();
        System.out.print("Working");
        if (driveSubsystem.isTrajectoryFinished()) {
            isFinished = true;
        }
    }
    
    /**
     * Stops all of the swerve modules located on the drivetrain. (All motor power is set to 0).
     */
    @Override
    public void end(boolean interrupted) {
        driveSubsystem.setX();
    }

    /**
     * Return whether or not the robot has finished.
     */
    @Override
    public boolean isFinished() {
        if (isFinished) {
            System.out.print("DOne");
        }
        return isFinished;
    }
}
