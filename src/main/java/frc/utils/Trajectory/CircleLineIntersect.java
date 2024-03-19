package frc.utils.Trajectory;

import java.util.List;

import edu.wpi.first.math.geometry.Translation2d;

public class CircleLineIntersect {
    
    /**
     * finds the intersections of a line onto a circle
     * @param firstSegmentEndpoint the first endpoint of the segment
     * @param secondSegmentEndpoint the second endpoint of the segment
     * @param circleCenter the center of the circle
     * @param circleRadius the radius of the circle
     * @return a list containing all of the intersection points (empty if there are none)
     */
    public static List<Translation2d> findIntersections(Translation2d firstSegmentEndpoint, Translation2d secondSegmentEndpoint, Translation2d circleCenter, double circleRadius) {
        
        //make sure that the line can be defined
        if (firstSegmentEndpoint.equals(secondSegmentEndpoint)) {
            return List.of();
        }

        //make sure that the circle can be defined
        if (circleRadius < 1e-6) {
            return List.of();
        }

        double x1 = firstSegmentEndpoint.getX() - circleCenter.getX();
        double y1 = firstSegmentEndpoint.getY() - circleCenter.getY();
        double x2 = secondSegmentEndpoint.getX() - circleCenter.getX();
        double y2 = secondSegmentEndpoint.getY() - circleCenter.getY();

        double dx = x2 - x1;
        double dy = y2 - y1;
        double dr = Math.sqrt(Math.pow(dx,2) + Math.pow(dy,2));
        double D = x1 * y2 - x2 * y1;

        double disc = Math.pow(circleRadius,2) * Math.pow(dr,2) - Math.pow(D,2);

        if (disc <= -1e-6) {
            return List.of();
        } else if (Math.abs(disc) < 1e-6) {
            double ix = (D * dy) / Math.pow(dr,2);
            double iy = - (D * dx) / Math.pow(dr,2);
            return List.of(new Translation2d(ix,iy));
        } else  {
            double ix1 = ( (D * dy) + (Math.signum(dy) * dx * Math.sqrt(disc)) ) / Math.pow(dr,2);
            double iy1 = - (  (D * dx) + (Math.abs(dy) * Math.sqrt(disc)) ) / Math.pow(dr,2);
            double ix2 = ( (D * dy) - (Math.signum(dy) * dx * Math.sqrt(disc)) ) / Math.pow(dr,2);
            double iy2 = - (  (D * dx) - (Math.abs(dy) * Math.sqrt(disc)) ) / Math.pow(dr,2);
            return List.of(new Translation2d(ix1,iy1),new Translation2d(ix2,iy2));
        }
    }

}
