package frc.robot.subsystems;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class BTOutakeSubsytem extends SubsystemBase{

    // Constants: will eventually be moved to constants class

    private final double OUTTAKE_POSITION_CONVERSION_FACTOR = 2 * Math.PI;
    private final double OUTTAKE_VELOCITY_CONVERSION_FACTOR = (2*Math.PI) / 60; 

    /*Fields Declaration

    We have a spark max and encoder for the left and right motors of our intake

    */
    private final CANSparkMax leftMotorSparkMax;
    private final CANSparkMax rightMotorSparkMax;
    private final RelativeEncoder leftMotorEncoder;
    private final RelativeEncoder rightMotorEncoder;
    private final SparkPIDController leftPIDController;
    private final SparkPIDController rightPIDController;

    // Main Constructor; Doesn't require any inputs
    public BTOutakeSubsytem(){


        //These are the field declarations
        leftMotorSparkMax = new CANSparkMax(Constants.BTConstants.CANIdConstants.LEFT_OUTTAKE_CAN_ID, MotorType.kBrushless);
        rightMotorSparkMax = new CANSparkMax(Constants.BTConstants.CANIdConstants.RIGHT_OUTTAKE_CAN_ID, MotorType.kBrushless);

        // Restoring the Spark Maxes to Factory Defaults for Consistency
        leftMotorSparkMax.restoreFactoryDefaults();
        rightMotorSparkMax.restoreFactoryDefaults();

        // Creating the Relative Encoder Objects
        leftMotorEncoder = leftMotorSparkMax.getEncoder();
        rightMotorEncoder = rightMotorSparkMax.getEncoder();

        // Creating the PID Controller Objects
        leftPIDController = leftMotorSparkMax.getPIDController();
        rightPIDController = rightMotorSparkMax.getPIDController();


        // Initializing the PID Controller Values
        leftPIDController.setP(Constants.BTConstants.PIDControllerConstants.P);
        leftPIDController.setD(Constants.BTConstants.PIDControllerConstants.D);
        leftPIDController.setI(Constants.BTConstants.PIDControllerConstants.I);
        leftPIDController.setFF(Constants.BTConstants.PIDControllerConstants.FF);
        leftPIDController.setOutputRange(Constants.BTConstants.PIDControllerConstants.MIN_OUTPUT, Constants.BTConstants.PIDControllerConstants.MAX_OUTPUT);
        leftPIDController.setSmartMotionMaxVelocity(Constants.BTConstants.PIDControllerConstants.OuttakeSubsystemConstants.MAX_VELOCITY, 0);
        leftPIDController.setSmartMotionMinOutputVelocity(Constants.BTConstants.PIDControllerConstants.OuttakeSubsystemConstants.MIN_VELOCITY, 0);
        leftPIDController.setSmartMotionMaxAccel(Constants.BTConstants.PIDControllerConstants.OuttakeSubsystemConstants.MAX_ACCELERATION, 0);
        leftPIDController.setSmartMotionAllowedClosedLoopError(Constants.BTConstants.PIDControllerConstants.ALLOWED_ERROR, 0);
        

        
        rightPIDController.setP(Constants.BTConstants.PIDControllerConstants.P);
        rightPIDController.setD(Constants.BTConstants.PIDControllerConstants.D);
        rightPIDController.setI(Constants.BTConstants.PIDControllerConstants.I);
        rightPIDController.setFF(Constants.BTConstants.PIDControllerConstants.FF);
        rightPIDController.setOutputRange(Constants.BTConstants.PIDControllerConstants.MIN_OUTPUT, Constants.BTConstants.PIDControllerConstants.MAX_OUTPUT);
        rightPIDController.setSmartMotionMaxVelocity(Constants.BTConstants.PIDControllerConstants.OuttakeSubsystemConstants.MAX_VELOCITY, 0);
        rightPIDController.setSmartMotionMinOutputVelocity(Constants.BTConstants.PIDControllerConstants.OuttakeSubsystemConstants.MIN_VELOCITY, 0);
        rightPIDController.setSmartMotionMaxAccel(Constants.BTConstants.PIDControllerConstants.OuttakeSubsystemConstants.MAX_ACCELERATION, 0);
        rightPIDController.setSmartMotionAllowedClosedLoopError(Constants.BTConstants.PIDControllerConstants.ALLOWED_ERROR, 0);

    }


    public void RunAtMaxSpeed(){

        /*
        The Method sets the Motors to Max Speed
        The negative value for the left spark max is because 
        the motors have to spin in opposite directions
         */ 

        leftPIDController.setReference(-3999, CANSparkMax.ControlType.kSmartVelocity);
        rightPIDController.setReference(3999, CANSparkMax.ControlType.kSmartVelocity);

    }


    public void RunMotorAtSpeed(double inputSpeed){

        leftPIDController.setReference(inputSpeed, CANSparkMax.ControlType.kSmartVelocity);
        rightPIDController.setReference(-inputSpeed, CANSparkMax.ControlType.kSmartVelocity);

    }

    public void StopMotor(){

        leftPIDController.setReference(0, CANSparkMax.ControlType.kSmartVelocity);
        rightPIDController.setReference(0, CANSparkMax.ControlType.kSmartVelocity);

    }

    public void outakeCommand(boolean outake) {

        if (outake){

            RunAtMaxSpeed();

        }
        else{
            StopMotor();
        }

    }
    
}
