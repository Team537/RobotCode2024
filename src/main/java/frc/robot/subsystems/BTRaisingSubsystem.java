package frc.robot.subsystems;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkAbsoluteEncoder.Type;
import com.revrobotics.SparkPIDController;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import frc.robot.IntakePosition;

import com.revrobotics.CANSparkLowLevel.MotorType;

public class BTRaisingSubsystem extends SubsystemBase{
    

    private CANSparkMax intakeRaiserSparkMax;
    private AbsoluteEncoder intakeRaiserEncoder;
    private SparkPIDController intakeRaiserPIDController;
    private IntakePosition intakePosition;

    /*
     * Constructs a New Intake Raiser Subsystem
     */
    public BTRaisingSubsystem(){

        /*
         * Raising Subsystem consists of:
         * A Spark Max
         * An Encoder
         * PID Controller
         */

        // Inititalization of Spark Max Object
        intakeRaiserSparkMax = new CANSparkMax(Constants.BTConstants.CANIdConstants.PIVOT_MOTOR_CAN_ID, MotorType.kBrushless);
        intakeRaiserSparkMax.restoreFactoryDefaults();
        

        // Initialization of Encoder & PID Controller Objects
        intakeRaiserEncoder = intakeRaiserSparkMax.getAbsoluteEncoder(Type.kDutyCycle);
        intakeRaiserEncoder.setVelocityConversionFactor(Constants.BTConstants.IntakeRaisingConstants.VELOCITY_CONVERSION_FACTOR);
        intakeRaiserEncoder.setPositionConversionFactor(Constants.BTConstants.IntakeRaisingConstants.POSITION_CONVERSION_FACTOR);

        intakeRaiserPIDController = intakeRaiserSparkMax.getPIDController();
        

        // PID Controller Constants Set Up
        intakeRaiserPIDController.setP(Constants.BTConstants.PIDControllerConstants.P);
        intakeRaiserPIDController.setD(Constants.BTConstants.PIDControllerConstants.D);
        intakeRaiserPIDController.setI(Constants.BTConstants.PIDControllerConstants.I);
        intakeRaiserPIDController.setFF(Constants.BTConstants.PIDControllerConstants.FF);
        intakeRaiserPIDController.setOutputRange(Constants.BTConstants.PIDControllerConstants.MIN_OUTPUT, Constants.BTConstants.PIDControllerConstants.MAX_OUTPUT);
        intakeRaiserPIDController.setSmartMotionMaxVelocity(Constants.BTConstants.PIDControllerConstants.RaisingSubsystemConstants.MAX_VELOCITY, 0);
        intakeRaiserPIDController.setSmartMotionMinOutputVelocity(Constants.BTConstants.PIDControllerConstants.RaisingSubsystemConstants.MIN_VELOCITY, 0);
        intakeRaiserPIDController.setSmartMotionMaxAccel(Constants.BTConstants.PIDControllerConstants.RaisingSubsystemConstants.MAX_ACCELERATION, 0);
        intakeRaiserPIDController.setSmartMotionAllowedClosedLoopError(Constants.BTConstants.PIDControllerConstants.ALLOWED_ERROR, 0);

    }

    public void goToIntakePosition(){

        intakeRaiserPIDController.setReference(Constants.BTConstants.IntakePositions.intakePosition, CANSparkMax.ControlType.kSmartMotion);
        intakePosition = IntakePosition.DOWN;
    }

    public void goToReleasePosition(){

        intakeRaiserPIDController.setReference(Constants.BTConstants.IntakePositions.releasePosition, CANSparkMax.ControlType.kSmartMotion);
        intakePosition = IntakePosition.UP;
    }

    public void periodic(){

        SmartDashboard.putNumber("Intake Raiser Position: ", intakeRaiserEncoder.getPosition());

    }

    public double intakePosition(){

        return intakeRaiserEncoder.getPosition();

    }


}
