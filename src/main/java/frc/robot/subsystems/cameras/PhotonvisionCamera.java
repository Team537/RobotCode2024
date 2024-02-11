package frc.robot.subsystems.cameras;

import java.util.List;

import org.photonvision.PhotonCamera;
import org.photonvision.targeting.MultiTargetPNPResult;
import org.photonvision.targeting.PNPResult;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class PhotonVisionCamera extends SubsystemBase {
    
    private PhotonCamera camera;
    private PhotonPipelineResult result;
    private Pose3d cameraOffset; // The camera's position relative to the robot.
    
    /**
     * Creates a <code> PhotonVisionCamera </code> with the desired paramaters.
     * 
     * @param cameraName The name of the camera you want to create.
     * @param cameraOffset The camera's position relative to the robot's origin.
     */
    public PhotonVisionCamera(String cameraName, Pose3d cameraOffset) {

        // Create a PhotonCamera object with the given name.
        this.camera = new PhotonCamera(cameraName);

        // Set the camera offset
        this.cameraOffset = cameraOffset;
    }

    @Override
    public void periodic() {

        // Update the camera's result so that all of our calculations are done using
        // the most recent data.
        result = camera.getLatestResult();
    }

    /**
     * Take a snapshot of the unprocessed camera feed. Useful for gathering training data for AI or
     * gathering interesting mid-match photographs. 
     */
    public void takeInputSnapshot() {
        camera.takeInputSnapshot();
    }

    /**
     * Take a snapshot of the processed camera feed. Useful for gathering interesting mid-match photographs. 
     */
    public void takeOutputSnapshot() {
        camera.takeOutputSnapshot();
    }

    /**
     * Returns the esimate position of the robot. 
     * 
     * @return The estimated position of the robot.
     */
    public Transform3d estimateRobotPose3d() {
        
        // Get the calculated MultiTagResult from this camera.
        MultiTargetPNPResult multiTargetPNPResults = result.getMultiTagResult();
        PNPResult estimatedPosition = multiTargetPNPResults.estimatedPose;

        // Make sure that the calcualted robot position isn't outdated
        if (!estimatedPosition.isPresent) {
            System.out.println("bad haha go brrrrrr");
            return null;
        }

        // TODO: Look into getting mroe accurate data.
        return estimatedPosition.best;
    }

    /**
     * This method sets the pipeline that this camera's stream will be processed using.
     * 
     * @param pipeline The pipeline number (0-9) you want this camera to be usinng 
     *                 to process the image.
     */
    public void setPipeline(int pipeline) {
        camera.setPipelineIndex(pipeline);
    }

    /**
     * Returns a PhotonTrackedTarget with the desired ID if this camera can see it. Otherwise
     * returns null.
     * 
     * @param id The ID of the tag you want to see if this camera can see.
     * @return If this camera can see a tag with the deisred ID then this method returns a 
     *         PhotonTrackedTarget. Otherwise this method returns null.
     */
    public PhotonTrackedTarget getDistacneToTag(int id) {

        // Make sure that this camera is detecting targets. If not then return false
        // since if there aren't any targets then there isn't a target with the drsired id.
        if(!hasTargets()) {
            return null;
        }

        // Get all of the visible targets and check if any of them have the desired ID. If they do,
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
     * This method returns a list containing information about all of the targets detected
     * by this camera.
     * 
     * @return A list of PhotonTrackedTargets containing all of the target's (Tags or notes)
     *          that this camera is currently detecting.
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
     * THis method returns the most prominent target visible by this camera.
     * 
     * @return The best target visible by this camera. (If there isn't a camer then this
     *          method returns null).
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
     * Returns whether or not this camera is currenlty viewing the desired apriltag.
     * 
     * @param id The ID of the AprilTag to be checked for visibility in the camera.
     * @return Whether or not an AprilTag with the desired ID is visible.
     */
    public boolean hasTargetWithId(int id) {

        // Make sure that this camera is detecting targets. If not then return false
        // since if there aren't any targets then there isn't a target with the drsired id.
        if(!hasTargets()) {
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
     * @return True if a target is visable, otherwise false.
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
     * Returns The name of this <code> PhotonVisionCamera </code>.
     * 
     * @return The name of this camera.
     */
    public String getCameraName() {
        return camera.getName();
    }
 }