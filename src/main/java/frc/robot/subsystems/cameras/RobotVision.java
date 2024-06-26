package frc.robot.subsystems.cameras;

import java.util.ArrayList;
import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.VisionConstants;
import frc.utils.geometry.EstimatedRobotPose3d;
import frc.utils.geometry.TagPose3d;

/**
 * An object used to manage and use data from cameras present on the robot.
 * 
 * @author Cameron Myhre
 * @version 1.0
 * @category Computer Vision
 */
public class RobotVision extends SubsystemBase {
    
    private final ArrayList<PhotonVisionCamera> photonVisionCameras;
    private final ArrayList<LimelightCamera> limelightCameras;

    private RobotVision(ArrayList<PhotonVisionCamera> photonVisionCameras, 
        ArrayList<LimelightCamera> limelightCameras) {

        // Set this RobotVision's processed cameras up.
        this.photonVisionCameras = photonVisionCameras;
        this.limelightCameras = limelightCameras;
    }

    public static class Builder {

        ArrayList<PhotonVisionCamera> photonVisionCameras = new ArrayList<>();
        ArrayList<LimelightCamera> limelightCameras = new ArrayList<>();

        /**
         * Creates a new <code> PhotonVisionCamera </code> object and add it to this 
         * <code> RobotVision </code> object's list of processed cameras.
         * 
         * @param cameraName The name of the camera you want to gain access to.
         * @param cameraOffset This camera's position relative to this robot's center.
         * @param pipeline The vision pipeline this camera will process images with.
         */
        public Builder addPhotonVisionCamera(String cameraName, Transform3d cameraOffset, int pipeline) {
            
            // Create a new camera with the desired settings
            PhotonVisionCamera newCamera = new PhotonVisionCamera(cameraName, cameraOffset);
            newCamera.setPipeline(pipeline);

            // Add the camera to the hashmap.
            this.photonVisionCameras.add(newCamera);

            // Make it possible to chain methods together.
            return this;
        }

        /**
         * Adds a <code> PhotonVisionCamera </code> object to this <code> RobotVision </code> object's 
         * list of processed cameras.
         * 
         * @param camera The camera that will be added to this <code> RobotVision </code> object's list
         *               of processed cameras.
         */
        public Builder addPhotonVisionCamera(PhotonVisionCamera camera) {

            // Add the camera to the hashmap.
            this.photonVisionCameras.add(camera);

            // Make it possible to chain methods together.
            return this;
        }

         /**
         * Create a new <code> LimelightCamera </code> object with the desired parameters and add it
         * to this <code> RobotVision </code>'s hashMap of processed limelight cameras.
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
         * @see LimelightCamera
         */
        public Builder addLimelightCamera(String networktableName, int ledMode, int camMode, int pipeline, int streamMode) {

            // Create a new limelight camera
            LimelightCamera newLimelight = new LimelightCamera(networktableName, ledMode, camMode, pipeline, streamMode);

            // Add the limelight to the list of limelight camera.
            limelightCameras.add(newLimelight);

            // Make it possible to chain methods together.
            return this;
        }

         /**
         * Adds a <code> LimelightCamera </code> object to this <code> RobotVision </code> object's 
         * list of processed cameras.
         * 
         * @param camera The camera that will be added to this <code> RobotVision </code> object's list
         *               of processed cameras.
         */
        public Builder addLimelightCamera(LimelightCamera camera) {

            // Add the limelight to the list of limelight camera.
            limelightCameras.add(camera);

            // Make it possible to chain methods together.
            return this;
        }

        /**
         * Builds the <code> RobotVision </code> objet.
         * 
         * @return A new <code> RobotVision </code> objet with the configured setting.
         */
        public RobotVision build() {
            return new RobotVision(this.photonVisionCameras, this.limelightCameras);
        }
    }

    @Override
    public void periodic() {

        /*
         * Try and estimate where the robot is on the field. Then, if we are able to successfully
         * estimate the robot's position, display the results on SmartDashboard so that we can determine
         * how accurate our estimation is.
         */
        Pose2d estimatedRobotPose = estimateRobotPose();
        if (estimatedRobotPose != null) { 
            SmartDashboard.putNumber("RobotVision's Estimated Robot X: ", estimatedRobotPose.getX());
            SmartDashboard.putNumber("RobotVision's Estimated Robot Y: ", estimatedRobotPose.getY());
            SmartDashboard.putNumber("RobotVision's Estimated Robot Heading: ", estimatedRobotPose.getRotation().getDegrees());
        }
    }

    /**
     * Estimates the robot's position on the field using by using the position data gathered from
     * visible AprilTags and returns the value. If the robot can't see any AprilTags, then this
     * method will return null.
     * 
     * @return The estimated position of the robot on the field as a {@code Pose2d}.
     */
    public Pose2d estimateRobotPose() {

        // Create a new ArrayList to store all of the estimated robot positions.
        EstimatedRobotPose3d estimatedRobotPosition = new EstimatedRobotPose3d();
        int numEstimatedPositions = 0;

        // Loop through all of this RobotVision's PhotonVisionCameras and get their estimated robot position 
        for (PhotonVisionCamera camera : photonVisionCameras) {

            // Get the camera's estimate of the robot's position on the field. If the camera was unable to estimate a
            // position, then skip over to the next camera.
            Optional<EstimatedRobotPose> optionalEstimatedPose = camera.estimateRobotPose();
            if (optionalEstimatedPose.isEmpty()) {
                continue;
            }

            // Get the EstimatedRobotPose from optionalEstimatedPosition since we know the value isn't null.  
            EstimatedRobotPose estimatedRobotPose = optionalEstimatedPose.get();

            // Get the estimated position and convert it to a EstimatedRobotPose3d. Then add it to estimatedRobotPosition
            // so that we can average out each camera's estimated position and get a more accurate estimate.
            EstimatedRobotPose3d estimatedRobotPose3d = new EstimatedRobotPose3d(estimatedRobotPose.estimatedPose);
            estimatedRobotPosition.add(estimatedRobotPose3d);

            // Add 1 to the numEstimatedPositions so that we can keep track of the total number of positions used in
            // the final estimate. This helps ensure our final position is as accurate as possible. 
            numEstimatedPositions++;
        }

         // Loop through all of this RobotVision's LimelightCameras and get their estimated robot position 
         for (LimelightCamera camera : limelightCameras) {

            // Get the camera's estimate of the robot's position on the field. If the camera was unable to estimate a
            // position, then skip over to the next camera.
            EstimatedRobotPose3d estimatedRobotPose3d = camera.estimateRobotPose3d();
            if (estimatedRobotPose3d == null) {
                continue;
            }

            // Add the camera's estimated position to estimatedRobotPosition so that we can average out each camera's 
            // estimated position and get a more accurate estimate.
            estimatedRobotPosition.add(estimatedRobotPose3d);

            // Add 1 to the numEstimatedPositions so that we can keep track of the total number of positions used in
            // the final estimate. This helps ensure our final position is as accurate as possible. 
            numEstimatedPositions++;
         }

        // Return null if we can't see any tags.
        if (numEstimatedPositions == 0) {
            return null;
        }

        // Average out all of the estimated values
        estimatedRobotPosition.div(numEstimatedPositions);

        return estimatedRobotPosition.toPose3d().toPose2d();
    }

    /**
     * Returns the distance to a desired tag if the robot can see said tag. If the tag is not visible, then
     * this method returns null.
     * 
     * @param id The ID fo the tag you want to get the distance to.
     * @return The distance to the desired tag as a <code> Transform3d </code>
     */
    public TagPose3d getDistanceToTag(int id) {

        ArrayList<PhotonTrackedTarget> validTargets = new ArrayList<>();
        TagPose3d tagPosition = new TagPose3d();

        // Loop through all of the cameras in this RobotVision object.
        for (PhotonVisionCamera camera : photonVisionCameras) {

            // Check whether or not the camera is currently viewing a AprilTag with the desired id. IF it is, then
            // ensure the the position data isn't ambiguous and add it to the list of validTargets for future mathematical operations.
            PhotonTrackedTarget target = validatePhotonTrackedTarget(camera, id);
            if (target != null) {
                validTargets.add(target);
            }
        }

        // Make sure that we were able to see at least 1 tag. If we weren't, then return, since we need tag data
        // to preform the following calculations.
        if (validTargets.size() == 0) {
            return null;
        }

        // Average out all of the tag data to minimize error.
        for (PhotonTrackedTarget target : validTargets) {
            tagPosition.plus(target.getBestCameraToTarget());
        }
        tagPosition.div(validTargets.size()); // Sets values equal to 1

        /*
         * Inverts the yaw of the TagPose3d object to align the robot's orientation with the detected tag.
         * This is necessary as the original yaw represents the tag's facing direction.
         */
        tagPosition.flipYaw();
        
        // Return the tag's location relative to the camera.
        return tagPosition;
    }

    /**
     * Returns a PhotonTrackedTarget if the specified camera is able to see a tag with a desired ID and 
     * provides unambiguous data. Otherwise return null.
     * 
     * @param camera The camera you want to check te targets of.
     * @param targetId The ID of the desired AprilTag target.
     * @return A PhotonTrackedTarget if the specified camera is able to see a tag with a desired ID and 
     *         provides unambiguous data. Otherwise return null.
     */
    private PhotonTrackedTarget validatePhotonTrackedTarget(PhotonVisionCamera camera, int targetId) {

        // Check if the camera is able to see the desired april tag. If not then return null.
        PhotonTrackedTarget target = camera.getDistanceToTag(targetId);
        if (target == null) {
            return null;
        }

        // Make sure the tag's ambiguous is lower than the maximum allowed ambiguity. This helps ensure that our robot
        // doesn't get confused by ambiguous position data.
        if (target.getPoseAmbiguity() > VisionConstants.MAX_AMBIGUITY) {
            return null;
        }

        // Return a PhotonTrackedTarget with the desired targetId.
        return target;
    }

    /**
     * Makes the camera with the desired name take a snapshot.
     * 
     * @param cameraName The name of the camera you want to have take a snapshot.
     */
    public void snapshot(String cameraName) {
                
        // Check if any of the PhotonVisionCameras have the desired name. If one does, then
        // take a snapshot and return.
        for (PhotonVisionCamera camera : photonVisionCameras) {
            if (camera.getCameraName() == cameraName) {
                camera.takeInputSnapshot();
                return;
            }
        }

        // Look through all of the limelight cameras and see if any of them have the desired name.
        // If one does, then take a snapshot and return.
        for (LimelightCamera camera : limelightCameras) {
            if (camera.getCameraName() == cameraName) {
                camera.snapshot();
                return;
            }
        }

        // No camera can be found with the desired name. Output an error to inform the user.
        System.err.println("Error: No camera exists with name \"" + cameraName + ".");
    }

    /**
     * Takes a snapshot on every camera.
     */
    public void snapshotAll() {

        // Loop through all of then cameras and take a snapshot (photograph) using each camera.
        for (PhotonVisionCamera camera : photonVisionCameras) {
            camera.takeInputSnapshot();
        }
        for (LimelightCamera camera : limelightCameras) {
            camera.snapshot();
        }
    }

    /**
     * Returns a {@code PhotonVisionCamera} object with the specified name.
     * 
     * @param cameraName The name of the {@code PhotonVisionCamera} you want to get.
     * @return A {@code PhotonVisionCamera} with the specified name. (Or null if no camera with the specified
     *         name can be found).
     */
    public PhotonVisionCamera getPhotonVisionCamera(String cameraName) {

        /*
         * Loop through all of this RobotVision's PhotonVisionCameras and attempt to find a camera with 
         * the specified name. If a camera with the specified name can be found, then return it. Otherwise 
         * look through the limelight cameras and try to find the camera there.
         */
        for (PhotonVisionCamera camera : photonVisionCameras) {

            // Check if the camera has the specified name and return it if it does.
            if (camera.getCameraName() == cameraName) {
                return camera;
            }
        }

        // Tell the user that no cameras with the given name could be found.
        System.err.println("Error: No camera found with name " + cameraName + ".");
        return null;
    }

    /**
     * Returns a {@code LimelightCamera} object with the specified name.
     * 
     * @param cameraName The name of the {@code LimelightCamera} you want to get.
     * @return A {@code LimelightCamera} with the specified name. (Or null if no camera with the specified
     *         name can be found).
     */
    public LimelightCamera getLimelightCamera(String cameraName) {

        /*
         * Loop through all of this RobotVision's LimelightCameras and attempt to find a camera with 
         * the specified name. If a camera with the specified name can be found, then return it. If 
         * no camera can be found then alert the return null.
         */
        for (LimelightCamera camera : limelightCameras) {

            // Check if the camera has the specified name and return it if it does.
            if (camera.getCameraName() == cameraName) {
                return camera;
            }
        }

        // Tell the user that no cameras with the given name could be found.
        System.err.println("Error: No camera found with name " + cameraName + ".");
        return null;
    }
}