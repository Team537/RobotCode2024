package frc.utils.Autonomous;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;

public enum AutonomousOption {

    RED_1  (new Pose2d(0, 0, new Rotation2d())),
    RED_2  (new Pose2d(0, 0, new Rotation2d())),
    RED_3  (new Pose2d(0, 0, new Rotation2d())),
    BLUE_1 (new Pose2d(0, 0, new Rotation2d())),
    BLUE_2 (new Pose2d(0, 0, new Rotation2d())),
    BLUE_3 (new Pose2d(0, 0, new Rotation2d()));

    private final Pose2d STARTING_LOCATION;

    /**
     * Initialize the STARTING_LOCATION variable for each of the enum values.
     * 
     * @param startingLocation The starting position of the robot.
     */
    AutonomousOption(Pose2d startingLocation) {
        this.STARTING_LOCATION = startingLocation;
    }

    /**
     * Returns the starting position of the robot.
     * 
     * @return The starting position of the robot as a {@code Pose2d}.
     */
    public Pose2d getStartingPosition() {
        return this.STARTING_LOCATION;
    }
}
