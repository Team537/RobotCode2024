package frc.utils.Autonomous;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;

public enum Alliance {

    RED (new Pose2d(new Translation2d(), Rotation2d.fromDegrees(0)), 
         new Pose2d(new Translation2d(), Rotation2d.fromDegrees(0)),
         Rotation2d.fromDegrees(180)),
    BLUE (new Pose2d(new Translation2d(), Rotation2d.fromDegrees(0)),
          new Pose2d(new Translation2d(), Rotation2d.fromDegrees(0)),
          Rotation2d.fromDegrees(0)),
    NOT_APPLICABLE (null,
          null,
          null);

    private final Pose2d AMP_POSITION;
    private final Pose2d SPEAKER_POSITION;
    private final Rotation2d ROTATIONAL_OFFSET;

    /**
     * Setup all values relevant to this alliance.
     * 
     * @param ampPosition The position of this alliance's amp.
     * @param speakerPosition The position of this alliance's speaker.
     * @param rotationalOffset A value required when driving the robot on red team, as we are using the field's 
     *                         coordinate system for the robot's orientation, which would cause our inputs to be 
     *                         reversed when driving. By setting this value to pi (180 degrees) wwe are able to flip
     *                         these inputs back to normal. 
     */
    Alliance(Pose2d ampPosition, Pose2d speakerPosition, Rotation2d rotationalOffset) {
        this.AMP_POSITION = ampPosition;
        this.SPEAKER_POSITION = speakerPosition;
        this.ROTATIONAL_OFFSET = rotationalOffset;
    }

    /**
     * Returns the position of this alliance's amp.
     * 
     * @return The position of this alliance's amp.
     */
    public Pose2d getAmpPosition() {
        return this.AMP_POSITION;
    }

    /**
     * Returns the position of this alliance's speaker.
     * 
     * @return The position of this alliance's speaker.
     */
    public Pose2d getSpeakerPosition() {
        return this.SPEAKER_POSITION;
    }

    /**
     * Returns this alliance's rotational offset.
     * 
     * @return A value required when driving the robot on red team, as we are using the field's 
     *         coordinate system for the robot's orientation, which would cause our inputs to be 
     *         reversed when driving. By setting this value to pi (180 degrees) wwe are able to flip
     *         these inputs back to normal. 
     */
    public Rotation2d getRotationalOffset() {
        return this.ROTATIONAL_OFFSET;
    }
}
