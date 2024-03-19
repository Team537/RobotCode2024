package frc.utils.Trajectory;

import java.util.List;

import org.opencv.core.Point;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;

/**
 * And object used to follow trajectories
 * 
 * @author Parker Huibregtse
 * @version 1.0
 * @category Trajectory
 */
public class PurePursuit {
    
    private List<Pose2d> path;
    private double currentWaypoint;
    private boolean targetOrientation;

    PurePursuit() {

    }

    public void followPath(Pose2d pose) {

        Translation2d target;
        boolean targetFound = false;

        //search through all paths for possible targets
        for (int i = 0; i < path.size() - 1; i++) {
            Translation2d point = path.get(i).getTranslation();
            Translation2d nextPoint = path.get(i + 1).getTranslation();
            List<Translation2d> intersections = CircleLineIntersect.findIntersections(point, nextPoint, pose.getTranslation(), 1);

            double furthestDistance = 0;
            Translation2d bestTarget = new Translation2d();
            boolean foundBestTarget = false;
            
            //find the maximum viable target
            for (int c = 0; c < intersections.size(); c++) {
                if (PointInsideRectangle.isInsideRectangle(intersections.get(c), point, nextPoint)) {
                    if (nextPoint.getDistance(intersections.get(c)) > furthestDistance) {
                        furthestDistance = nextPoint.getDistance(intersections.get(c));
                        bestTarget = intersections.get(c);
                        foundBestTarget = true;
                    }
                }
            }

            if (foundBestTarget) {
                target = bestTarget;
                targetFound = true;
            }
        }

        if (!targetFound) {

        } else {
            
        }

    }

}
