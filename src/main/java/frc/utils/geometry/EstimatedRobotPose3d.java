package frc.utils.geometry;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;

/**
 * Represents the robot's estimated position on the field.
 * 
 * @author Cameron Myhre
 * @version 1.1
 * @category Position data types.
 */
public class EstimatedRobotPose3d {

    private double x, y, z, roll, pitch, yaw;

    /**
     * Creates an empty {@code RobotPose3d} object with all values set to 0.
     */
    public EstimatedRobotPose3d() {
        this.x = 0;
        this.y = 0;
        this.z = 0;
        this.roll = 0;
        this.pitch = 0;
        this.yaw = 0;
    }

    /**
     * Creates a new {@code EstimatedRobotPose3d} object with the specified values.
     * 
     * @param x         The robot's x coordinate.
     * @param y         The robot's y coordinate.
     * @param z         The robot's z coordinate.
     * @param roll      The roll angle in radians.
     * @param pitch     The pitch angle in radians.
     * @param yaw       The yaw angle in radians.
     */
    public EstimatedRobotPose3d(double x, double y, double z, double roll, double pitch, double yaw) {
        this.x = x;
        this.y = y;
        this.z = z;
        this.roll = roll;
        this.pitch = pitch;
        this.yaw = yaw;
    }

    /**
     * Creates a new {@code EstimatedRobotPose3d} object with the specified values.
     * 
     * @param x         The robot's x coordinate.
     * @param y         The robot's y coordinate.
     * @param z         The robot's z coordinate.
     * @param rotation  A {@code Rotation3d} specifying which direction the robot is facing.\
     */
    public EstimatedRobotPose3d(double x, double y, double z, Rotation3d rotation) {
        this.x = x;
        this.y = y;
        this.z = z;
        this.roll = rotation.getX();
        this.pitch = rotation.getY();
        this.yaw = rotation.getZ();
    }

    /**
     * Creates a new {@code EstimatedRobotPose3d} with the specified position and rotation.
     * 
     * @param position A {@code Translation3d} specifiying where the robot is located on the field.
     * @param rotation A {@code Rotaiton3d} specifying which direciton the robot is facing.
     */
    public EstimatedRobotPose3d(Translation3d position, Rotation3d rotation) {
        this.x = position.getX();
        this.y = position.getY();
        this.z = position.getZ();
        this.roll = rotation.getX();
        this.pitch = rotation.getY();
        this.yaw = rotation.getZ();
    }

    /**
     * Creates a new {@code EstimatedRobotPose3d} object using the values of a {@code Transform3d}.
     * 
     * @param robotPosition The {@code Transform3d} whose values you want to clone to this 
     *                      {@code EstimatedRobotPose3d}.
     */
    public EstimatedRobotPose3d(Transform3d robotPosiion) {
        this.x = robotPosiion.getX();
        this.y = robotPosiion.getY();
        this.z = robotPosiion.getZ();

        // Get the rotation from the Transform3d oject and clone it's values to this 
        // EstimatedRobotPose3d's values.
        Rotation3d rotation = robotPosiion.getRotation();
        this.roll = rotation.getX();
        this.pitch = rotation.getY();
        this.yaw = rotation.getZ();
    }

    /**
     * Create a new {@code EstimatedRobotPose3d} object using values from a specified {@code Pose3d}.
     * 
     * @param robotPosiion The {@code Pose3d} object whoes values you want to clone to this
     *                     {@code EstimatedRobotPose3d}.
     */
    public EstimatedRobotPose3d(Pose3d robotPosiion) {
        this.x = robotPosiion.getX();
        this.y = robotPosiion.getY();
        this.z = robotPosiion.getZ();

        // Get the rotation from the Pose3d oject and clone it's values to this 
        // EstimatedRobotPose3d's values.
        Rotation3d rotation = robotPosiion.getRotation();
        this.roll = rotation.getX();
        this.pitch = rotation.getY();
        this.yaw = rotation.getZ();
    }

    /**
     * Returns the distance to the specified location as a {@code Pose3d} object.
     * 
     * @param position The position you want to get the distance to.
     * @return The distance to the specified location as {@code Pose3d} object.
     */
    public Pose3d getDistanceTo(EstimatedRobotPose3d position) {
        double relativeX = position.getX() - this.x;
        double relativeY = position.getY() - this.y;
        double relativeZ = position.getZ() - this.z;

        double relativeRoll = position.getRoll() - this.roll;
        double relativePitch = position.getPitch() - this.pitch;
        double relativeYaw = position.getYaw() - this.yaw;

        return new Pose3d(
            new Translation3d(relativeX, relativeY, relativeZ),
            new Rotation3d(relativeRoll, relativePitch, relativeYaw)
        );
    }

    /**
     * Returns a {@code Rotation3d} object specifying how much this robot would have to rotate
     * to face the same direction.
     * 
     * @param rotation The rotation you want to get the distance to.
     * @return A {@code Pose3d} object specifying the amount that this robot would have to rotate
     *         in each 
     */
    public Pose3d getDistanceTo(Rotation3d rotation) {
        double relativeRoll = rotation.getX() - this.roll;
        double relativePitch = rotation.getY() - this.pitch;
        double relativeYaw = rotation.getZ() - this.yaw;

        return new Pose3d(
            new Translation3d(0, 0, 0),
            new Rotation3d(relativeRoll, relativePitch, relativeYaw)
        );
    }

    /**
     * Returns the distance this robot would have to travel in order to reach the specified location.
     * 
     * @param position The position you want to get the distance to.
     * @return The distance to the specified location as a {@code Pose3d} object.
     */
    public Pose3d getDistanceTo(Translation3d position) {
        double relativeX = position.getX() - this.x;
        double relativeY = position.getY() - this.y;
        double relativeZ = position.getZ() - this.z;

        return new Pose3d(
            new Translation3d(relativeX, relativeY, relativeZ),
            new Rotation3d(0, 0, 0)
        );
    }

    /**
     * Returns a {@code Pose3d} object specifying how much the robot will have to drive an rotate along 
     * each axis in order to reach the specified position. 
     * 
     * @param position The position you want to get the distance to.
     * @return The distance to the specified position as a {@code Pose3d} object.
     */
    public Pose3d getDistanceTo(Pose3d position) {
        double relativeX = position.getX() - this.x;
        double relativeY = position.getY() - this.y;
        double relativeZ = position.getZ() - this.z;

        Rotation3d rotation = position.getRotation(); 
        double relativeRoll = rotation.getX() - this.roll;
        double relativePitch = rotation.getY() - this.pitch;
        double relativeYaw = rotation.getZ() - this.yaw;

        return new Pose3d(
            new Translation3d(relativeX, relativeY, relativeZ),
            new Rotation3d(relativeRoll, relativePitch, relativeYaw)
        );
    }

    /**
     * Returns a new {@code Pose3d} object containing the values of this {@code EstimatedRobotPose3d}.
     * 
     * @return A new {@code Pose3d} object containing the values of this {@code EstimatedRobotPose3d}.
     */
    public Pose3d toPose3d() {
        return new Pose3d(
            getTranslation3d(), 
            getRotation());
    }

    /**
     * Adds the specified {@code EstimatedRobotPose3d}'s values to this 
     * {@code EstimatedRobotPose3d}'s values.
     * 
     * @param robotPose The {@code EstimatedRobotPose3d} who's values are to be added to this 
     *                  {@code EstimatedRobotPose3d}'s values.
     */
    public void add(EstimatedRobotPose3d robotPose) {
        this.x += robotPose.getX();
        this.y += robotPose.getY();
        this.z += robotPose.getZ();
        this.roll += robotPose.getRoll();
        this.pitch += robotPose.getPitch();
        this.yaw += robotPose.getYaw();
    }

    /**
     * Adds the specified {@code Pose3d}'s values to this 
     * {@code EstimatedRobotPose3d}'s values.
     * 
     * @param robotPose The {@code Pose3d} who's values are to be added to this 
     *                  {@code EstimatedRobotPose3d}'s values.
     */
    public void add(Pose3d pose3d) {
        this.x += pose3d.getX();
        this.y += pose3d.getY();
        this.z += pose3d.getZ();

        // Get the rotation from the specified Pose3d and add its values to this RobotPose3d's values.
        Rotation3d rotation = pose3d.getRotation();
        this.roll += rotation.getX();
        this.pitch += rotation.getY();
        this.yaw += rotation.getZ();
    }

    /**
     * Subtracts the specified {@code EstimatedRobotPose3d}'s values from this 
     * {@code EstimatedRobotPose3d}'s values.
     * 
     * @param robotPose The {@code EstimatedRobotPose3d} who's values are to be subtracted from this 
     *                  {@code EstimatedRobotPose3d}'s values.
     */
    public void minus(EstimatedRobotPose3d robotPose) {
        this.x -= robotPose.getX();
        this.y -= robotPose.getY();
        this.z -= robotPose.getZ();
        this.roll -= robotPose.getRoll();
        this.pitch -= robotPose.getPitch();
        this.yaw -= robotPose.getYaw();
    }

    /**
     * Subtracts the specified {@code Pose3d}'s values from this 
     * {@code EstimatedRobotPose3d}'s values.
     * 
     * @param robotPose The {@code EstimatedRobotPose3d} who's values are to be subtracted from this 
     *                  {@code EstimatedRobotPose3d}'s values.
     */
    public void minus(Pose3d pose3d) {
        this.x -= pose3d.getX();
        this.y -= pose3d.getY();
        this.z -= pose3d.getZ();

        // Get the rotation from the specified Pose3d and add its values to this RobotPose3d's values.
        Rotation3d rotation = pose3d.getRotation();
        this.roll -= rotation.getX();
        this.pitch -= rotation.getY();
        this.yaw -= rotation.getZ();
    }

     /**
     * Multiplies all values of this {@code EstimatedRobotPose3d} by the specified multiplier.
     * 
     * @param multiplier The value by which all values of this {@code EstimatedRobotPose3d} will be multiplied 
     *                   by.
     */
    public void times(double multiplier) {
        this.x *= multiplier;
        this.y *= multiplier;
        this.z *= multiplier;
        this.roll *= multiplier;
        this.pitch *= multiplier;
        this.yaw *= multiplier;
    }
    
    /**
     * Divides all values of this {@code EstimatedRobotPose3d} by the specified divisor.
     * 
     * @param divisor The value by which all values of this {@code EstimatedRobotPose3d} will 
     * be divided by.
     */
    public void div(double divisor) {
        this.x /= divisor;
        this.y /= divisor;
        this.z /= divisor;
        this.roll /= divisor;
        this.pitch /= divisor;
        this.yaw /= divisor;
    }

    /**
     * Sets this {@code EstimatedRobotPose3d}'s x coordinate to the specified value.
     * 
     * @param newX The new x coordinate of this {@code EstimatedRobotPose3d} in meters.
     */
    public void setX(double newX) {

    }

    /**
     * Sets this {@code EstimatedRobotPose3d}'s y coordinate to the specified value.
     * 
     * @param newY The new y coordiante of this {@code EstimatedRobotPose3d} in meters.
     */
    public void setY(double newY) {
        this.y = newY;
    }

    /**
     * Sets this {@code EstimatedRobotPose3d}'s z coordinate to the specified value.

     * @param newZ The new z coordiante of this {@code EstimatedRobotPose3d} in meters.
     */
    public void setZ(double newZ) {
        this.z = newZ;
    }

    /**
     * Sets this {@code EstimatedRobotPose3d}'s roll angle to the specified value.
     * 
     * @param newRoll The new roll angle of this {@code EstimatedRobotPose3d} in radians.
     */
    public void setRoll(double newRoll) {
        this.roll = newRoll;
    }

    /**
     * Sets this {@code EstimatedRobotPose3d}'s pitch angle to the specified value.
     * 
     * @param newPitch The new pitch angle of this {@code EstimatedRobotPose3d} in radians.
     */
    public void setPitch(double newPitch) {
        this.pitch = newPitch;
    }

    /**
     * Sets this {@code EstimatedRobotPose3d}'s yaw angle to the specified value.
     * 
     * @param newYaw The new yaw angle of this {@code EstimatedRobotPose3d} in radians.
     */
    public void setYaw(double newYaw) {
        this.yaw = newYaw;
    }

    /**
     * Returns a {@code Translation3d} object containing this {@code EstimatedRobotPose3d} object's 
     * positional values.
     * 
     * @return A {@code Translation3d} object containing this {@code EstimatedRobotPose3d} object's 
     *         positional values.
     */
    public Translation3d getTranslation3d() {
        return new Translation3d(x, y, z);
    }

    /**
     * Returns the x coordiante of this {@code EstimatedRobotPose3d}.
     * 
     * @return The x coordiante of this {@code EstimatedRobotPose3d}.
     */
    public double getX() {
        return this.x;
    }

    /**
     * Returns the y coordiante of this {@code EstimatedRobotPose3d}.
     * 
     * @return The y coordiante of this {@code EstimatedRobotPose3d}.
     */
    public double getY() {
        return this.y;
    }

    /**
     * Returns the z coordiante of this {@code EstimatedRobotPose3d}.
     * 
     * @return The z coordiante of this {@code EstimatedRobotPose3d}.
     */
    public double getZ() {
        return this.z;
    }

    /**
     * Returns a new {@code Rotation3d} containing this {@code EstimatedRobotPose3d}'s rotation.
     * 
     * @return A new {@code Rotation3d} containing this {@code EstimatedRobotPose3d}'s rotation.
     */
    public Rotation3d getRotation() {
        return new Rotation3d(this.roll, this.pitch, this.yaw);
    }

    /**
     * Returns the roll angle of this {@code EstimatedRobotPose3d} in radians.
     * 
     * @return The roll angle of this {@code EstimatedRobotPose3d} in radians.
     */
    public double getRoll() {
        return this.roll;
    }

    /**
     * Returns the pitch angle of this {@code EstimatedRobotPose3d} in radians.
     * 
     * @return The pitch angle of this {@code EstimatedRobotPose3d} in radians.
     */
    public double getPitch() {
        return this.pitch;
    }

    /**
     * Returns the yaw angle of this {@code EstimatedRobotPose3d} in radians.
     * 
     * @return The yaw angle of this {@code EstimatedRobotPose3d} in radians.
     */
    public double getYaw() {
        return this.yaw;
    }
}