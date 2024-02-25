package frc.robot;
import java.lang.Math;

public class OuttakeAngleCalculator {
    
    //Constants

    private final double A;
    private final double B;
    private final double L;
    private final double K;
    private final double r;


    public OuttakeAngleCalculator(){

        // These constants used for the math were measured in CAD, but will be remeasured in
        // real life

        A = 3.65; // This is the initial x position 
        B = 2; // This is the initial y position
        L = 8.5236; // This is the length of the longer arm
        K = 5.5; // This is the length of the shorter arm
        r = 2; // This is the radius of turnage


    }

    private double radiansToDegrees(double angleInRadians){

        return Math.toDegrees(angleInRadians);

    }

    // These methods are used to make the code more readable
    private double sin(double angle){
        return Math.sin(angle);
    }

    private double cos(double angle){
        return Math.cos(angle);
    }

    private double squared(double value){

        return Math.pow(value, 2);

    }

    // This method calculates the angle of the outtake based on the angle that the
    // two bar linkage has turned.
    // It uses a formula created earlier by Vahe Ohihoin, linked in the black team discord.
    // The values phi, alpha, and R are defined by that formula

    public double outtakeAngleinRadians(double gamma){

        // gamma is the angle that the two-bar linkage has turned

        // This converts the angle in degrees to radians
        gamma = gamma * (Math.PI / 180);
        double outtakeAngle;

        double R = Math.sqrt(
            squared( A + r*sin(gamma) ) + squared( B - r*(1-cos(gamma)) )
        );

        double phi = Math.atan(
            ( B - r*(1 - cos(gamma)) ) / ( A + r*sin(gamma) )
        );

        double alpha = Math.acos(
        
            ( squared(L) + squared(R) - squared(K) ) / ( 2 * L * R )
        
        );

        outtakeAngle = phi + alpha;
        
        return outtakeAngle;

    }

    public double outtakeAngleInDegrees(double gamma){

        return radiansToDegrees(outtakeAngleinRadians(gamma));

    } 


}
