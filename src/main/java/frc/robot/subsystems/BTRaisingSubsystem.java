package frc.robot.subsystems;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkAbsoluteEncoder.Type;
import com.revrobotics.SparkPIDController;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.IntakePosition;
import edu.wpi.first.wpilibj.Timer;

public class BTRaisingSubsystem extends SubsystemBase{
    

    private CANSparkMax intakeRaiserSparkMax;
    private AbsoluteEncoder intakeRaiserAbsoluteEncoder;
    private RelativeEncoder intakeRaiserRelativeEncoder;
    private SparkPIDController intakeRaiserPIDController;
    private IntakePosition intakePosition = IntakePosition.UP;
    private Timer newTimer = new Timer();

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
        intakeRaiserAbsoluteEncoder = intakeRaiserSparkMax.getAbsoluteEncoder(Type.kDutyCycle);
        intakeRaiserRelativeEncoder = intakeRaiserSparkMax.getEncoder();

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

        intakeRaiserPIDController.setReference(-12.65, CANSparkMax.ControlType.kSmartMotion);
        intakePosition = IntakePosition.DOWN;
    }

    public void goToReleasePosition(){

        intakeRaiserPIDController.setReference(0, CANSparkMax.ControlType.kSmartMotion);
        intakePosition = IntakePosition.UP;
    }

    public void periodic(){

        // SmartDashboard.putNumber("Intake Raiser Position: ", intakeRaiserAbsoluteEncoder.getPosition());
        SmartDashboard.putString("Intake Position", intakePosition.toString());


    }

    public double intakePosition(){

        return intakeRaiserAbsoluteEncoder.getPosition();

    }

    public void goToAmpPosition(){

        intakeRaiserPIDController.setReference(Constants.BTConstants.IntakePositions.ampScoringPosition, CANSparkMax.ControlType.kSmartMotion);
        intakePosition = IntakePosition.AMP;

    }

    public void raisingCommand(boolean ampScoring, boolean raise) {

        if (ampScoring){

            goToAmpPosition();

        }

        else if (raise){

            if (intakePosition == IntakePosition.UP){

                Timer.delay(0.2);
                goToIntakePosition();
            }
            else if (intakePosition == IntakePosition.DOWN){

                Timer.delay(0.2);
                goToReleasePosition();
            }

        }
    }
}
