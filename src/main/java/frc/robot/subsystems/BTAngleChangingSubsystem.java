package frc.robot.subsystems;

import edu.wpi.first.wpilibj.ADIS16448_IMU;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.OuttakeAngleCalculator;
import frc.robot.Constants.BTConstants;
import frc.robot.subsystems.cameras.RobotVision;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.SparkAbsoluteEncoder.Type;

public class BTAngleChangingSubsystem extends SubsystemBase{

    /*
     * This system uses the following objects:
     * Spark Maxes
     * Absolute Encoders
     * Relative Encoders
     * PID Controller
     * Angle Calculator Object Using a class I created
     */
    private CANSparkMax angleChangerSparkMax;
    private AbsoluteEncoder angleChangerAbsoluteEncoder;
    private RelativeEncoder angleChangerRelativeEncoder;
    private SparkPIDController angleChangerPIDController;
    private OuttakeAngleCalculator angleCalc;

    /*
    This is the system used to get the shooter angle to change
    * Since we use kSmartMotion to change our angle, our reference value is in rotations from starting
    * The increaseAngle() method increases the new reference rotation by the rotationStep  
    * everytime it's called. This causes the number of rotations of the MOTOR
    * to increase by the rotation step.
    * The decreaseAngle() method works the same way in reverse
    */
    private double angle = 0;
    private double rotationStep = 0.1;

    // Constructor which initializes the objects
    public BTAngleChangingSubsystem(RobotVision robotVisionObject){

        // This sparkmax controls the outtake angle motor
        angleChangerSparkMax = new CANSparkMax(Constants.BTConstants.CANIdConstants.ANGLE_CHANGER_CAN_ID, MotorType.kBrushless);
        angleChangerSparkMax.restoreFactoryDefaults();

        // This gets the absolute and relative encoder for the spark max
        // The absolute encoder values are mostly used for calculations and positioning
        angleChangerAbsoluteEncoder = angleChangerSparkMax.getAbsoluteEncoder(Type.kDutyCycle);

        angleChangerRelativeEncoder = angleChangerSparkMax.getEncoder();

        // Creating the PID Controller Object and Configuring it
        angleChangerPIDController = angleChangerSparkMax.getPIDController();
        angleChangerPIDController.setP(Constants.BTConstants.PIDControllerConstants.P);
        angleChangerPIDController.setD(Constants.BTConstants.PIDControllerConstants.D);
        angleChangerPIDController.setI(Constants.BTConstants.PIDControllerConstants.I);
        angleChangerPIDController.setFF(Constants.BTConstants.PIDControllerConstants.FF);
        angleChangerPIDController.setOutputRange(Constants.BTConstants.PIDControllerConstants.MIN_OUTPUT, Constants.BTConstants.PIDControllerConstants.MAX_OUTPUT);
        angleChangerPIDController.setSmartMotionMaxVelocity(Constants.BTConstants.PIDControllerConstants.AngleChangingConstants.MAX_VELOCITY, 0);
        angleChangerPIDController.setSmartMotionMinOutputVelocity(Constants.BTConstants.PIDControllerConstants.AngleChangingConstants.MIN_VELOCITY, 0);
        angleChangerPIDController.setSmartMotionMaxAccel(Constants.BTConstants.PIDControllerConstants.AngleChangingConstants.MAX_ACCELERATION, 0);
        angleChangerPIDController.setSmartMotionAllowedClosedLoopError(Constants.BTConstants.PIDControllerConstants.ALLOWED_ERROR, 0);

        // This assigns a robot vision object to the angle calculator robot to use for
        // calulations
        angleCalc = new OuttakeAngleCalculator(robotVisionObject);

    }

    // These methods increase and decrease the angle when called.
    public void decreaseAngle(){

        angle += rotationStep;
        angleChangerPIDController.setReference(angle, CANSparkMax.ControlType.kSmartMotion);

    }

    public void increaseAngle(){

        // The if statement makes sure that the angle never goes below zero
        if (angle - rotationStep > 0){

            angle -= rotationStep;
            angleChangerPIDController.setReference(angle, CANSparkMax.ControlType.kSmartMotion);

        }


    }




    // This will be used to automatically set the angle when we get distance data from vision.
    public void approachAngle(double desiredAngle){

        double difference = desiredAngle - outtakeAngleInDegrees();
        double allowedError = 2; // this is in degrees

        if (difference > allowedError){
            increaseAngle();
        }
        else if (difference < -allowedError){
            decreaseAngle();
        }

    }



    public void periodic(){

        // SmartDashboard.putNumber("Angle Changer Angle Turned: ", encoderOutputInDegrees() );
        // SmartDashboard.putNumber("Shooter Angle: ", outtakeAngleInDegrees() );
        // SmartDashboard.putBoolean("SCORE FAR: ", ( Math.abs(outtakeAngleInDegrees() - BTConstants.AnglingConstants.scoreFarAngle) <= 2 ));
        // SmartDashboard.putBoolean("SCORE CLOSE: ",  Math.abs(outtakeAngleInDegrees() - BTConstants.AnglingConstants.scoreCloseAngle) <= 2 );

    }

    // These create the encoder and shooter angle values that are used for calculations

    private double encoderOutputInDegrees(){

        //Absolute Encoder outputs in rotation
        // This converts it to degrees
        return ( angleChangerAbsoluteEncoder.getPosition() * 360 );

    }

    private double outtakeAngleInDegrees(){

        // This takes the angle that the link turns and converts it to the angle of the outtake

        return( angleCalc.outtakeAngleInDegrees(encoderOutputInDegrees()) );

    }


    // This runs as the default command for the angling subsystem

    public void defaultAnglingCommand(boolean closeScore, boolean farScore, boolean ampScoring){

        if (closeScore){
            angleChangerPIDController.setReference((0), CANSparkMax.ControlType.kSmartMotion);
        }
        else if (farScore){

            angleChangerPIDController.setReference(5, CANSparkMax.ControlType.kSmartMotion);
        }

        else if(ampScoring){

            angleChangerPIDController.setReference(0 , CANSparkMax.ControlType.kSmartMotion);

        }

    

    }



}