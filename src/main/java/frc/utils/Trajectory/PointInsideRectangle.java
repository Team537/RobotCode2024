package frc.utils.Trajectory;

import edu.wpi.first.math.geometry.Translation2d;

//fun fact: this file was written entirely by chat-gpt
public class PointInsideRectangle {

    /**
     * Checks if a Translation2d point is inside a rectangle defined by two other Translation2d vertices.
     *
     * @param point       The Translation2d point to check.
     * @param rectVertex1 One of the vertices of the rectangle.
     * @param rectVertex2 The opposite vertex of the rectangle.
     * @return True if the point is inside the rectangle, false otherwise.
     */
    public static boolean isInsideRectangle(Translation2d point, Translation2d rectVertex1, Translation2d rectVertex2) {
        double minX = Math.min(rectVertex1.getX(), rectVertex2.getX());
        double maxX = Math.max(rectVertex1.getX(), rectVertex2.getX());
        double minY = Math.min(rectVertex1.getY(), rectVertex2.getY());
        double maxY = Math.max(rectVertex1.getY(), rectVertex2.getY());

        double pointX = point.getX();
        double pointY = point.getY();

        return pointX >= minX && pointX <= maxX && pointY >= minY && pointY <= maxY;
    }
}
