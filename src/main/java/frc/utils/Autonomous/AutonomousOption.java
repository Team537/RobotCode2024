package frc.utils.Autonomous;

import java.util.List;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.Constants.AutoConstants;

public enum AutonomousOption {

    RED_1  (AutoConstants.RED_1_TRAJECTORY, 
            AutoConstants.RED_1_STARTING_POSE, 
            Alliance.RED),
    RED_2  (AutoConstants.RED_2_TRAJECTORY, 
            AutoConstants.RED_2_STARTING_POSE, 
            Alliance.RED),
    RED_3  (AutoConstants.RED_3_TRAJECTORY, 
            AutoConstants.RED_3_STARTING_POSE, 
            Alliance.RED),
    BLUE_1 (AutoConstants.BLUE_1_TRAJECTORY, 
            AutoConstants.BLUE_1_STARTING_POSE,
            Alliance.BLUE),
    BLUE_2 (AutoConstants.BLUE_2_TRAJECTORY, 
            AutoConstants.BLUE_2_STARTING_POSE,
            Alliance.BLUE),
    BLUE_3 (AutoConstants.BLUE_3_TRAJECTORY, 
            AutoConstants.BLUE_3_STARTING_POSE,
            Alliance.BLUE);

    private final Pose2d STARTING_LOCATION;
    private final List<Pose2d> TRAJECTORY;
    private final Alliance ALLIANCE;
    private final Pose2d AMP_POSITION;
    private final Pose2d SPEAKER_POSITION;
    private final Rotation2d TELEOP_ROTATION_OFFSET;

    /**
     * Initialize the STARTING_LOCATION variable for each of the enum values.
     * 
     * @param trajectory A list of points that the robot will drive to during auto.
     * @param startingLocation The starting position of the robot.
     * @param alliance 
     */
    AutonomousOption(List<Pose2d> trajectory, Pose2d startingLocation, Alliance alliance) {
        this.TRAJECTORY = trajectory;
        this.STARTING_LOCATION = startingLocation;
        this.ALLIANCE = alliance;

        this.AMP_POSITION = ALLIANCE.getAmpPosition();
        this.SPEAKER_POSITION = ALLIANCE.getSpeakerPosition();
        this.TELEOP_ROTATION_OFFSET = ALLIANCE.getRotationalOffset();
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
     * Returns this at autonomous path's trajectory, which is a list of {@Code Pose3d} that the robot will travel to.
     * 
     * @return This autonomous path's trajectory.
     */
    public List<Pose2d> getTrajectory() {
        return this.TRAJECTORY;
    }

    /**
     * Returns the alliance that this auto is associated with.
     * 
     * @return The alliance that this auto is associated with.
     */
    public Alliance getAlliance() {
        return this.ALLIANCE;
    }

    /**
     * Returns the position of this auto's target amp.
     * 
     * @return The position of this auto's target amp.
     */
    public Pose2d getAmpPosition() {
        return this.AMP_POSITION;
    }

    /**
     * Returns the position of this auto's target speaker.
     * 
     * @return The position of this auto's target speaker.
     */
    public Pose2d getSpeakerPosition() {
        return this.SPEAKER_POSITION;
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
