package frc.utils.geometry;

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;

/**
 * The {@code TagPose3d} class represents the 3D pose of an AprilTag relative to a camera.
 * It provides method to help manage and use the rotational and positional data provided by 
 * these detected tags.
 * 
 * @author Cameron Myhre
 * @version 1.0
 */
public class TagPose3d {
    private double x, y, z, roll, yaw, pitch;

    /**
     * Creates an empty {@code TagPose3d} object with all values set to 0.
     */
    public TagPose3d() {
        this.x = 0;
        this.y = 0;
        this.z = 0;
        this.roll = 0;
        this.pitch = 0;
        this.yaw = 0;
    }

    /**
     * Creates a {@code TagPose3d} object with specified values.
     *
     * @param x     The distance (left/right) of the camera from the tag.
     * @param y     The distance (forwards/backwards) of the camera from the tag.
     * @param z     The distance (above/below) of the camera from the tag.
     * @param roll  The roll angle.
     * @param pitch The pitch angle.
     * @param yaw   The yaw angle.
     */
    public TagPose3d(double x, double y, double z, double roll, double pitch, double yaw) {
        this.x = x;
        this.y = y;
        this.z = z;
        this.roll = roll;
        this.pitch = pitch;
        this.yaw = yaw;
    }

    /**
     * Creates a {@code TagPose3d} object with specified values and rotation.
     *
     * @param x        The distance (left/right) of the robot from the tag.
     * @param y        The distance (forwards/backwards) of the robot from the tag.
     * @param z        The distance (above/below) of the robot from the tag.
     * @param rotation The rotation in 3D space.
     */
    public TagPose3d(double x, double y, double z, Rotation3d rotation) {
        this.x = x;
        this.y = y;
        this.z = z;
        this.roll = rotation.getX();
        this.pitch = rotation.getY();
        this.yaw = rotation.getZ();

    }
    
    /**
     * Creates a {@code TagPose3d} object by cloning values from a {@code Transform3d} object.
     *
     * @param transform3d The {@code Transform3d} whose values you want to clone.
     */
    public TagPose3d (Transform3d transform3d) {
        this.x = transform3d.getX();
        this.y = transform3d.getY();
        this.z = transform3d.getZ();
        this.roll = transform3d.getRotation().getX();
        this.pitch = transform3d.getRotation().getY();
        this.yaw = transform3d.getRotation().getZ();
    }

    /**
     * Divides all values of this {@code TagPose3d} object by the specified divisor.
     *
     * @param divisor The value by which all values of this {@code TagPose3d} object will be divided.
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
     * Multiplies all values of this {@code TagPose3d} object by the specified multiplier.
     * 
     * @param value The value by which all values of this {@code TagPose3d} object will be multplied by.
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
     * Adds all the values of the specified {@code Transform3d} object to this {@code TagPose3d} object.
     * 
     * @param transform3d The {@code Transform3d} whose values add to this {@code TagPose3d}.
     */
    public void plus(Transform3d transform3d) {

        // Add all of the position values from the provided Transform3d object to this
        // TagPose3d's position values.
        this.x += transform3d.getX();
        this.y += transform3d.getY();
        this.z += transform3d.getZ();

        // Get the provided Transform3d object's rotation and add all values to this TagPose3d
        // object's rotation values.
        Rotation3d rotation3d = transform3d.getRotation();
        this.roll += rotation3d.getX();
        this.pitch += rotation3d.getY();
        this.yaw += rotation3d.getZ();
    }

    /**
     * Subtracts all the values of the specified {@code Transform3d} object from this {@code TagPose3d} object.
     * 
     * @param transform3d The {@code Transform3d} whose values subtract from this {@code TagPose3d}'s values.
     */
    public void minus(Transform3d transform3d) {

        // Subtract the provided Transform3d object's position values from this TagPose3d object's
        // positional values.
        this.x -= transform3d.getX();
        this.y -= transform3d.getY();
        this.z -= transform3d.getZ();

        // Get the provided transform3d's rotation and then subtract the provided rotation values 
        // from this TagPose3d object's rotation values.
        Rotation3d rotation3d = transform3d.getRotation();
        this.roll -= rotation3d.getX();
        this.pitch -= rotation3d.getY();
        this.yaw -= rotation3d.getZ();
    }

    /**
     * Sets the X-coordinate of this TagPose3d to the specified value.
     *
     * @param newX The new X-coordinate.
     */
    public void setX(double newX) {
        this.x = newX;
    }

    /**
     * Sets the Y-coordinate of this TagPose3d to the specified value.
     *
     * @param newY The new Y-coordinate.
     */
    public void setY(double newY) {
        this.y = newY;
    }

    /**
     * Sets the Z-coordinate of this TagPose3d to the specified value.
     *
     * @param newZ The new Z-coordinate.
     */
    public void setZ(double newZ) {
        this.z = newZ;
    }

    /**
     * Sets the rotation of this TagPose3d using the provided Rotation3d object.
     *
     * @param newRotation The new rotation in 3D space.
     */
    public void setRotation(Rotation3d newRotation) {
        this.roll = newRotation.getX();
        this.pitch = newRotation.getY();
        this.yaw = newRotation.getZ();
    }

    /**
     * Sets the roll angle of this TagPose3d to the specified value.
     *
     * @param newRoll The new roll angle.
     */
    public void setRoll(double newRoll) {
        this.roll = newRoll;
    }

    /**
     * Sets the pitch angle of this TagPose3d to the specified value.
     *
     * @param newPitch The new pitch angle.
     */
    public void setPitch(double newPitch) {
        this.pitch = newPitch;
    }

    /**
     * Sets the yaw angle of this TagPose3d to the specified value.
     *
     * @param newYaw The new yaw angle.
     */
    public void setYaw(double newYaw) {
        this.yaw = newYaw;
    }

    /**
     * Rotates each angle by 180 degrees.
     */
    public void flipAngle() {
        flipRoll();
        flipPitch();
        flipYaw();
    }


    /**
     * Rotates roll by 180 degrees.
     */
    public void flipRoll() {
        this.roll -= Math.signum(this.roll) * Math.PI;
    }

    /**
     * Rotates pitch by 180 degrees.
     */
    public void flipPitch() {
        this.pitch -= Math.signum(this.pitch) * Math.PI;
    }
    
    /**
     * Rotates yaw by 180 degrees.
     */
    public void flipYaw() {
        this.yaw -= Math.signum(this.yaw) * Math.PI;
    }

    /**
     * Returns the X position of this {@code TagPose3d}.
     * 
     * @return The x position of this {@code TagPose3d}.
     */
    public double getX() {
        return this.x;
    }

    /**
     * Returns how far forwards the camera would have to move to reach the tag.
     * 
     * @return How far forwards the camera would have to move to reach the tag. 
     */
    public double getY() {
        return this.y;
    }

    /**
     * Returns how much higher/lower the camera would need to be to be level with the tag.
     * 
     * @return how much higher/lower the camera would need to be in order to be level with the tag.
     */
    public double getZ() {
        return this.z;
    }

    /**
     * Returns a {@code Rotation3d} object containing values describing how much the camera would need
     * to rotate along each axis in order to be facing the same direction as the tag.
     * 
     * @return A {@code Rotation3d} object containing values describing how much the camera would need
     *         to rotate along each axis in order to be facing the same direction as the tag.
     */
    public Rotation3d getRotation() {
        return new Rotation3d(this.roll, this.pitch, this.yaw);
    }

    /**
     * Returns the roll of this TagPose.
     * 
     * @return The roll (rotation along the x axis) of this TagPose in radians.
     */
    public double getRoll() {
        return this.roll;
    }

    /**
     * Returns the pitch of this TagPose.
     * 
     * @return The pitch (rotation along the y axis) of this TagPose in radians.
     */
    public double getPitch() {
        return this.pitch;
    }

    /**
     * Returns the yaw of this TagPose.
     * 
     * @return The yaw (rotation along the z axis) of this TagPose in radians.
     */
    public double getYaw() {
        return this.yaw;
    }

    /**
     * Returns the distance to the tag
     * 
     * @return The point to point horizontal planar distance of the camera to the tag
     * 
     */

    public double getDistance(){

        return Math.sqrt((x*x) + (y*y));
    }

}