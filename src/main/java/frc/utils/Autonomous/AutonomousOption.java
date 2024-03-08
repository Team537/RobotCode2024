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

    AutonomousOption(Pose2d startingLocation) {
        this.STARTING_LOCATION = startingLocation;
    }
}
