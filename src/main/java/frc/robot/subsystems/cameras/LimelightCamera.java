package frc.robot.subsystems.cameras;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.utils.geometry.EstimatedRobotPose3d;

/**
 * A Limelight 3 camera. 
 * 
 * @author Cameron Myhre
 * @version 2.0
 * @category Computer Vision
 */
public class LimelightCamera extends SubsystemBase {
    
    private NetworkTable table;
    private String name; // NetworkTable's name

    /**
     * Create a new LimelightCamera object with the desired parameters.
     * 
     * @param networktableName The name of the network table that this limelight is using.
     * @param ledMode The LEDMode you want this limelight to use. 
     *                  <ul>
     *                      <li> 0: Use the LED Mode set in the current pipeline </li>
     *                      <li> 1: Force off </li>
     *                      <li> 2: Force blink </li>
     *                      <li> 3: Force on </li>
     *                  </ul>
     * @param camMode The operating mode you want this limelight to be running in.
     *                  <ul>
     *                      <li> 0: Vision processor </li>
     *                      <li> 1: Driver Camera (Increases exposure, disables vision processing) </li>
     *                  </ul>
     * @param pipeline The pipeline (number 0-9) that you want this limelight to use.
     * @param streamMode The streaming mode you want this limelight to be running in.
     *                  <ul>
     *                      <li> 0: Standard - Side-by-side streams if a webcam is attached to Limelight </li>
     *                      <li> 1: PiP Main - The secondary camera stream is placed in the lower-right corner of the primary camera stream </li>
     *                      <li> 2: PiP Secondary - The primary camera stream is placed in the lower-right corner of the secondary camera stream </li>
     *                  </ul>
     */
    public LimelightCamera(String networktableName, int ledMode, int camMode, int pipeline, int streamMode) {
        table = NetworkTableInstance.getDefault().getTable(networktableName);

        // Configure settings
        name = networktableName;
        setLedMode(ledMode);
        setCameraMode(camMode);
        setPipeline(pipeline);
        setStreamMode(streamMode);
    }

    @Override
    public void periodic() {}
    
    /**
     * Sets this limelight's LED's mode.
     *  
     * @param mode The mode you want this limelight's LED's to be set to. Below is a list
     *                of the LED modes along with which input will set the LEDs to each mode.
     *                  <ul>
     *                      <li> 0: Use the LED Mode set in the current pipeline </li>
     *                      <li> 1: Force off </li>
     *                      <li> 2: Force blink </li>
     *                      <li> 3: Force on </li>
     *                  </ul>
     */
    public void setLedMode(int mode) {

        // Make sure mode is within the allowed range.
        mode = validateAndClampInput(mode, 0, 3, "LED Mode");

        // Set this limelight's LED Mode.
        getValue("ledMode").setNumber(mode);
    }

    /**
     * Sets this limelight's camera mode.
     * 
     * @param mode The operating mode you want this limelight to be running in.
     *                  <ul>
     *                      <li> 0: Vision processor </li>
     *                      <li> 1: Driver Camera (Increases exposure, disables vision processing) </li>
     *                  </ul>
     */
    public void setCameraMode(int mode) {

        // Make sure mode is within the allowed range.
        mode = validateAndClampInput(mode, 0, 1, "Camera Mode");

        // Set this limelight's camera mode.
        getValue("camMode").setNumber(mode);
    }

    /**
     * Sets this limelight's streaming mode.
     * 
     * @param mode The streaming mode you want this limelight to be running in.
     *              <ul>
     *                  <li> 0: Standard - Side-by-side streams if a webcam is attached to Limelight </li>
     *                  <li> 1: PiP Main - The secondary camera stream is placed in the lower-right corner of the primary camera stream </li>
     *                  <li> 2: PiP Secondary - The primary camera stream is placed in the lower-right corner of the secondary camera stream </li>
     *              </ul>
     */
    public void setStreamMode(int mode) {

        // Make sure mode is within the allowed range.
        mode = validateAndClampInput(mode, 0, 2, "Camera Mode");

        // Set this limelight's streaming mode.
        getValue("stream").setNumber(mode);
    }

    /**
     * Sets this limelight's pipeline. 
     * 
     * @param pipeline The pipeline (number 0-9) that you want this limelight to use.
     */
    public void setPipeline(int pipeline) {
        getValue("pipeline").setNumber(pipeline);
    }

    /**
     * Take exactly one snapshot.
     */
    public void snapshot() {
        getValue("snapshot").setNumber(1);
    }

    /**
     * Reset snapshot mode.
     */
    public void resetSnapshotMode() {
        getValue("snapshot").setNumber(0);
    }

    /**
     * Returns which pipeline this limelight is currently using.
     * 
     * @return The pipeline that this limelight is currently using.
     */
    public double getPipeline() {
        return getValue("pipeline").getDouble(1);
    }

    /**
     * This method returns true if the limelight can see an AprilTag.
     * 
     * @return Whether or not an april tag is being detected.
     */
    public boolean hasTarget() {
        return getValue("tv").getDouble(0) == 1;
    }

    /**
     * This method returns the ID of the most prominent AprilTag. If there isn't a visible tag,
     * then this method will return -1.
     * 
     * @return The id of the most prominent AprilTag. If there isn't a tag in view, then return -1.
     */
    public double getPrimaryTagId() {
        return getValue("tid").getDouble(-1);
    }

    /**
     * This method returns the horizontal offset from the crosshair to the target.
     * 
     * @return Horizontal offset from crosshair to target in degrees. (Output ranges from +- 1/2 FOV)
     */
    public double getTx() {
        return getValue("tx").getDouble(0);
    }

    /**
     * This method returns the vertical offset from the crosshair to the target.
     * 
     * @return Vertical Offset from crosshair to target in degrees. (Output ranges from +- 1/2 FOV)
     */
    public double getTy() {
        return getValue("ty").getDouble(0);
    }

    /**
     * This method returns this limelight's pipeline's latency in ms.
     * 
     * @return This limelight's pipeline's latency in ms.
     */
    public double getLatency() {
        return getValue("tl").getDouble(0);
    }

    /**
     * This method returns the position on the field that the robot thinks the robot is 
     * located at as a <code> Pose3d </code>.
     * 
     * @return The robot's position on the field as a {@code EstimatedRobotPose3d}.
     */
    public EstimatedRobotPose3d estimateRobotPose3d() {

        // Get an array containing the values of the robot's calculated position on the field.
        double[] botposeValues = getValue("botpose").getDoubleArray(new double[6]);

        // Return an EstimatedRobotPose3d containing the camera's estimate of the robot's position on the field.
        return new EstimatedRobotPose3d(
            botposeValues[0], botposeValues[1], botposeValues[2], // Robot's position on the filed (x, y, z) in meters.
            botposeValues[3], botposeValues[4], botposeValues[5]  // Robot's rotation (roll, pitch, yaw) in radians.
            );
    }

    /**
     * Returns the name of the camera.
     * 
     * @return The name of this camera.
     */
    public String getCameraName() {
        return name;
    }

    /**
     * Returns the NetworkTableEntry under this Limelight's NetworkTable with the 
     * desired name (key).
     * 
     * @param key The name of the NetworkTableEntry you want to get.
     * @return The specified NetworkTableEntry.
     */
    private NetworkTableEntry getValue(String key) {
        return table.getEntry(key);
    }

    /**
     * Checks if the provided value is within the desired range. If it isn't, then print an
     * error and set the value to the closest value in the range. Lastly, return the value.
     * 
     * @param value The value to clamp and validate.
     * @param min The minimum allowed value.
     * @param max The maximum allowed value.
     * @param settingName The name of the setting that is trying to be set/changed.
     * @return A value that is within the desired range as close to the original value as possible.
     */
    private int validateAndClampInput(int value, int min, int max, String settingName) {

        // Make sure the value is within the desired range. If it isn't, then print out an error and 
        // set value to the closest value within the range.
        if (value < min) {
            System.err.println("Error: " + settingName + " out of range. " +
            settingName + " cannot be less than " + min + ". Received: " + value);
            value = min;
        } else if (value > max) {
            System.err.println("Error: " + settingName + " out of range. " +
            settingName + " cannot be greater than " + max + ". Received: " + value);
            value = max;
        }

        // Return a value that we know is within the desired range.
        return value;
    }
}