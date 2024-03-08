package frc.robot.subsystems.cameras;

import java.util.List;
import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.VisionConstants;

/**
 * A camera attatched to a co-processer.
 * 
 * @author Cameron Myhre
 * @version 1.0
 * @category Computer Vision
 */
public class PhotonVisionCamera extends SubsystemBase {

    private PhotonCamera camera;
    private Transform3d cameraOffset; // The camera's position relative to the robot.

    private PhotonPipelineResult result;
    private PhotonPoseEstimator photonPoseEstimator;

    /**
     * Creates a {@code PhotonVisionCamera} with the desired parameters.
     * 
     * @param cameraName   The name of the camera you want to create.
     * @param cameraOffset The camera's position relative to the robot's origin.
     */
    public PhotonVisionCamera(String cameraName, Transform3d cameraOffset) {

        // Create a PhotonCamera object with the given name.
        this.camera = new PhotonCamera(cameraName);

        // Set the camera offset
        this.cameraOffset = cameraOffset;

        // Initialize this camera's PhotonPoseEstimaotr so that we are able to estimate
        // the robot's position.
        photonPoseEstimator = new PhotonPoseEstimator(VisionConstants.APRIL_TAG_FIELD_LAYOUT,
                PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, camera, cameraOffset);
    }

    @Override
    public void periodic() {

        // Update the camera's result so that all of our calculations are done using
        // the most recent data.
        result = camera.getLatestResult();

        /*
         * Output values so that I, Cameron, can figure out how photonvision's Object
         * Detection code
         * works. This needs to be done because there isn't any documentation, and the
         * people on the discord
         * server effectibly told me to just figure it out.
         */
        if (result.hasTargets()) {

            if(result.getTargets().isEmpty()) {
                System.out.println("Work");
                return;
            }

            System.out.println(result.getTargets().size());
            SmartDashboard.putNumber("Area", result.getTargets().get(0).getArea());
            SmartDashboard.putString("Id", String.valueOf(result.getTargets().get(0).getFiducialId()));

            if (result.targets.size() > 1) {
                SmartDashboard.putString("Id 1", String.valueOf(result.getTargets().get(1).getFiducialId()));
            }
            SmartDashboard.putNumber("Ambiguity", result.getTargets().get(0).getPoseAmbiguity());
            SmartDashboard.putNumber("Skew", result.getTargets().get(0).getSkew());
            SmartDashboard.putNumber("Pitch", result.getTargets().get(0).getPitch());
            SmartDashboard.putNumber("Yaw", result.getTargets().get(0).getYaw());

            SmartDashboard.putString("Class ", String.valueOf(result.getTargets().get(0).getClass()));

            SmartDashboard.putString("To String Target", result.targets.get(0).toString());
        }
    }

    /**
     * Take a snapshot of the unprocessed camera feed. Useful for gathering training
     * data for AI or
     * gathering interesting mid-match photographs.
     */
    public void takeInputSnapshot() {
        // camera.takeInputSnapshot();
    }

    /**
     * Take a snapshot of the processed camera feed. Useful for gathering
     * interesting mid-match photographs.
     */
    public void takeOutputSnapshot() {
        // camera.takeOutputSnapshot();
    }

    /**
     * Estimates the robot's position on the filed using all of the visilbe
     * AprilTags and returns the result.
     * 
     * @return An estimante of the robot's positon on the field.
     */
    public Optional<EstimatedRobotPose> estimateRobotPose() {
        return photonPoseEstimator.update();
    }

    /**
     * This method sets the pipeline that this camera's stream will be processed
     * using.
     * 
     * @param pipeline The pipeline number (0-9) you want this camera to be using
     *                 to process the image.
     */
    public void setPipeline(int pipeline) {
        camera.setPipelineIndex(pipeline);
    }

    /**
     * Returns a PhotonTrackedTarget with the desired ID if this camera can see it.
     * Otherwise
     * returns null.
     * 
     * @param id The ID of the tag you want to see if this camera can see.
     * @return If this camera can see a tag with the desired ID then this method
     *         returns a
     *         PhotonTrackedTarget. Otherwise this method returns null.
     */
    public PhotonTrackedTarget getDistanceToTag(int id) {

        // Make sure that this camera is on the ApilTag detection pipeline. If it isn't,
        // then print an error and return null.
        if (camera.getPipelineIndex() != VisionConstants.APRIL_TAG_PIPELINE) {

            // Print out an error informing the programmer that they may have made a
            // mistake.
            System.err.println("Error: " + camera.getName() + " attempted to detect AprilTag using non-ApriLTag " +
                    "pipeline. Current pipeline  number is " + camera.getPipelineIndex() + ".");

            // Stop running the rest of the code, since we know that we won't find any april
            // tags.
            return null;
        }

        // Make sure that this camera is detecting targets. If not then return false
        // since if there aren't any targets then there isn't a target with the desired
        // id.
        if (!hasTargets()) {
            return null;
        }

        // Get all of the visible targets and check if any of them have the desired ID.
        // If they do,
        // then return the target.
        List<PhotonTrackedTarget> targets = getTargets();
        for (PhotonTrackedTarget target : targets) {

            // Check if the target has the desired ID.
            if (target.getFiducialId() == id) {
                return target;
            }
        }

        // This camera can't see a tag with the desired id.
        return null;
    }

    /**
     * This method returns a list containing information about all of the targets
     * detected
     * by this camera.
     * 
     * @return A list of PhotonTrackedTargets containing all of the target's (Tags
     *         or notes)
     *         that this camera is currently detecting.
     */
    public List<PhotonTrackedTarget> getTargets() {

        // Check whether or not the camera is getting any detections
        if (!hasTargets()) {
            return null;
        }

        // Return all of the detected targets.
        return result.getTargets();
    }

    /**
     * This method returns the most prominent target visible by this camera.
     * 
     * @return The best target visible by this camera. (If there isn't a camera then
     *         this
     *         method returns null).
     */
    public PhotonTrackedTarget getPrimaryTarget() {

        // Check whether or not the camera is getting any detections
        if (!hasTargets()) {
            return null;
        }

        // Return the primary target
        return result.getBestTarget();
    }

    /**
     * Returns whether or not this camera is currently viewing the desired AprilTag.
     * 
     * @param id The ID of the AprilTag to be checked for visibility in the camera.
     * @return Whether or not an AprilTag with the desired ID is visible.
     */
    public boolean hasTargetWithId(int id) {

        // Make sure that this camera is detecting targets. If not then return false
        // since if there aren't any targets then there isn't a target with the desired
        // id.
        if (!hasTargets()) {
            return false;
        }

        // Get all of the visible targets and check if any of them have the desired ID.
        List<PhotonTrackedTarget> targets = getTargets();
        for (PhotonTrackedTarget target : targets) {

            // Check if the target has the desired ID.
            if (target.getFiducialId() == id) {
                return true;
            }
        }

        // There isn't a target with the desired ID.
        return false;
    }

    /**
     * This method returns whether or not this camera is detecting any targets.
     * 
     * @return True if a target is visible, otherwise false.
     */
    public boolean hasTargets() {
        return result.hasTargets();
    }

    /**
     * This method returns the latency of the pipeline is milliseconds.
     * 
     * @return The latency of the pipeline is milliseconds.
     */
    public double getLatencyMs() {
        return result.getLatencyMillis();
    }

    /**
     * Returns The name of this {@code PhotonVisionCamera}.
     * 
     * @return The name of this camera.
     */
    public String getCameraName() {
        return camera.getName();
    }
}