package frc.robot.subsystems.cameras;

import java.util.ArrayList;

import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CameraConstants;
import frc.utils.TagPose3d;

public class RobotVision extends SubsystemBase {
    
    private final ArrayList<PhotonVisionCamera> photonVisionCameras;
    private final ArrayList<LimelightCamera> limelightCameras;

    private RobotVision(ArrayList<PhotonVisionCamera> photonVisionCameras, 
        ArrayList<LimelightCamera> limelightCameras) {

        // Set this RobotVision
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
         * @param cameraName The name of the camera you want to gain acsess to.
         * @param cameraOffset This camera's position relative to this robot's center.
         * @param pipeline The vision pipeline this camera will process images with.
         */
        public Builder addPhotonVisionCamera(String cameraName, Pose3d cameraOffset, int pipeline) {
            
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
         * Create a new <code> LimelightCamera </code> object with the desired paramaters and add it
         * to this <code> RobotVision </code>'s hashMap of processed limelight cameras.
         * 
         * @param networktableName The anme of the network table that this limelight is using.
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

            // Create a new limelgith camera
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
         * @return A new <code> RobotVision </code> objet with the configured setting.
         */
        public RobotVision build() {
            return new RobotVision(this.photonVisionCameras, this.limelightCameras);
        }
    }

    @Override
    public void periodic() {}

    public Pose3d estimateRobotPose() { return null; }

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
        
        // Display values on dashboard
        SmartDashboard.putNumber("TagX", tagPosition.getX());
        SmartDashboard.putNumber("TagY", tagPosition.getY());
        SmartDashboard.putNumber("TagZ", tagPosition.getZ());
        SmartDashboard.putNumber("roll", tagPosition.getRotation().getX());
        SmartDashboard.putNumber("pitch", tagPosition.getRotation().getY());
        SmartDashboard.putNumber("yaw", tagPosition.getRotation().getZ());

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
        if (target.getPoseAmbiguity() > CameraConstants.MAX_AMBIGUITY) {
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
    public void snapshot() {

        // Loop through all of then cameras and take a snapshot (photograph) using each camera.
        for (PhotonVisionCamera camera : photonVisionCameras) {
            camera.takeInputSnapshot();
        }
        for (LimelightCamera camera : limelightCameras) {
            camera.snapshot();
        }
    }
}