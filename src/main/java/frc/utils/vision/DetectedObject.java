package frc.utils.vision;

import java.util.List;

import org.photonvision.targeting.PhotonTrackedTarget;
import org.photonvision.targeting.TargetCorner;

public class DetectedObject {
    
    private double x, y;
    private double area, percentOfScreen;
    private double skew;

    /**
     * Create a {@code DetectedObject} using values from a {@code PhotonTrackedTarget}.
     * 
     * @param detectedObject The {@code PhotonTrackedTarget} who's values will be used to create
     *                       this {@code DetectedObject}
     */
    public DetectedObject(PhotonTrackedTarget detectedObject) {

        // Get the detected object's area and position on the screen.
        double[] screenPosition = calculateScreenPosition(detectedObject);
        this.x = screenPosition[0];
        this.y = screenPosition[1];
        this.area = screenPosition[2];

        // Get the object's skew 
        this.skew = detectedObject.getSkew();
        this.percentOfScreen = detectedObject.getArea();
    }

    /**
     * Calculates the object's area and position on the screen.
     * 
     * @param detectedObject The PhotonTrackedTarget who's values will be used to preform the position
     *                       calculations.
     * @return An array of doubles containing the object's position on the screen and area it take sup, in pixels.
     *         {x, y, area}
     */
    private double[] calculateScreenPosition(PhotonTrackedTarget detectedObject) {
        
        // Get all of the detected object's corners. This allows us to use the corners' position to 
        // figure out where the object is on the screen.
        List<TargetCorner> targetCorners  = detectedObject.getMinAreaRectCorners();
        TargetCorner bottomLeftCorner = targetCorners.get(0);
        TargetCorner topRightCorner = targetCorners.get(2);
        
        /*
         * Add the x and y coordinates of both corners together and divide the result by two. Doing this
         * results in the output being exactly halfway between the two corners, which jut so happens to be 
         * the center of the object.
         */
        double x = (bottomLeftCorner.x + topRightCorner.x) / 2;
        double y = (bottomLeftCorner.y + topRightCorner.y) / 2;

        /*
         * Then, since our captured image is (1280 x 720p) and the image's origin is in the top left corner, we 
         * subtract half of this from our calculated coordinates, as doing so sets our origin as the center which 
         * makes everything easier to work with.
         */
        x -= 640; 
        y -= 360;
        y *= -1;

        // Calculate the length and width of the object's bounding box and multiply them together to find the area.
        double area = Math.abs(topRightCorner.x - bottomLeftCorner.x) * 
            Math.abs(topRightCorner.y - bottomLeftCorner.y);
        
        // Return an array containing the above calculated x and y coordinates and area.
        double[] fieldPosition = {x, y, area};
        return fieldPosition;
    }

    /**
     * Returns the X coordinate of this detected object (Screen position, in pixels).
     * 
     * @return The X coordinate of this detected object (Screen position, in pixels).
     */
    public double getX() {
        return this.x;
    }

    /**
     * Returns the Y coordinate of this detected object (Screen position, in pixels).
     * 
     * @return The Y coordinate of this detected object (Screen position, in pixels).
     */
    public double getY() {
        return this.y;
    }

    /**
     * Returns the s% of the screen that this object takes up.
     * 
     * @return The space on the screen that this object's bounding box takes up, in pixels.
     */
    public double getArea() {
        return this.area;
    }

    /**
     * Returns the percent of the screen taken up by this object (0.0 -> 1.0).
     * 
     * @return The percent of the screen taken up by this object (0.0 -> 1.0).
     */
    public double getPercentOfScreen() {
        return this.percentOfScreen;
    }

    /**
     * Returns this object's skew.
     * 
     * @return This object's skew.
     */
    public double getSkew() {
        return this.skew;
    }
}
