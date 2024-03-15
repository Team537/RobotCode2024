package frc.utils.Autonomous;

import java.util.List;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.Constants.AutoConstants;

public enum AutonomousOption {

    RED_1  (AutoConstants.RED_1_TRAJECTORY, 
            new Pose2d(0, 0, new Rotation2d()), 
            new Rotation2d(Math.PI)),
    RED_2  (AutoConstants.RED_2_TRAJECTORY, 
            new Pose2d(0, 0, new Rotation2d()),
            new Rotation2d(Math.PI)),
    RED_3  (AutoConstants.RED_3_TRAJECTORY, 
            new Pose2d(0, 0, new Rotation2d()), 
           new Rotation2d(Math.PI)),
    BLUE_1 (AutoConstants.BLUE_1_TRAJECTORY, 
            new Pose2d(0, 0, new Rotation2d()), 
            new Rotation2d(0)),
    BLUE_2 (AutoConstants.BLUE_2_TRAJECTORY, 
            new Pose2d(0, 0, new Rotation2d()), 
            new Rotation2d(0)),
    BLUE_3 (AutoConstants.BLUE_3_TRAJECTORY, 
            new Pose2d(0, 0, new Rotation2d()), 
            new Rotation2d(0));

    private final Pose2d STARTING_LOCATION;
    private final List<Pose2d> TRAJECTORY;
    private final Rotation2d TELEOP_ROTATION_OFFSET;

    /**
     * Initialize the STARTING_LOCATION variable for each of the enum values.
     * 
     * @param trajectory A list of points that the robot will drive to during auto.
     * @param startingLocation The starting position of the robot.
     * @param teleopRotationOffset A value required when driving the robot on red team, as we are using the field's 
     *                             coordinate system for the robot's orientation, which would cause our inputs to be 
     *                             reversed when driving. By setting this value to pi (180 degrees) wwe are able to flip
     *                             these inputs back to normal.
     */
    AutonomousOption(List<Pose2d> trajectory, Pose2d startingLocation, Rotation2d teleopRotationOffset) {
        this.TRAJECTORY = trajectory;
        this.STARTING_LOCATION = startingLocation;
        this.TELEOP_ROTATION_OFFSET = teleopRotationOffset;
    }

    /**
     * Returns the starting position of the robot.
     * 
     * @return The starting position of the robot as a {@code Pose2d}.
     */
    public Pose2d getStartingPosition() {
        return this.STARTING_LOCATION;
    }

    /**
     * Returns this at authonomous path's trajectory, which is a list of {@Code Pose3d} that the robot will travel to.
     * 
     * @return This authonomous path's trajectory.
     */
    public List<Pose2d> getTrajectory() {
        return this.TRAJECTORY;
    }

    /**
     * Returns a the value by which the robot's rotation will be offset by, so that our drive inputs won't be
     *         reversed on red alliance.
     * 
     * @return A the value by which the robot's rotation will be offset by, so that our drive inputs won't be
     *         reversed on red alliance.
     */
    public Rotation2d getTeleopRotationalOffset() {
        return this.TELEOP_ROTATION_OFFSET;
    }
}
