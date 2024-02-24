package frc.robot.subsystems;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkLimitSwitch;
import com.revrobotics.SparkPIDController;
import edu.wpi.first.wpilibj.Timer;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.IntakePosition;

/*
The Intake Subsystem controls the Mechanism for Intaking the Note
It has a:
 * Spark Max
 * Limit Switch 
 * Relative Encoder
 * PID Controller
 */

public class BTIntakeSubsytem extends SubsystemBase{


    // Field Declaration

    private final CANSparkMax intakeSparkMax;
    private final RelativeEncoder intakeRelativeEncoder;
    private final SparkLimitSwitch limitSwitch;
    private final SparkPIDController intakePIDController;
    private String intakeToggleString = "intaking";
    private Timer newTimer = new Timer();
    private IntakePosition intakePosition;

    private boolean buttonPressed = false;

    // Constructor
    public BTIntakeSubsytem(){

        // Initializations

        // Spark Max
        intakeSparkMax = new CANSparkMax(Constants.BTConstants.CANIdConstants.INTAKE_MOTOR_CAN_ID, MotorType.kBrushless);
        intakeSparkMax.restoreFactoryDefaults();

        // Relative Encoder
        intakeRelativeEncoder = intakeSparkMax.getEncoder();
        

        // Limit Switch
        limitSwitch = intakeSparkMax.getForwardLimitSwitch(SparkLimitSwitch.Type.kNormallyOpen);

        // PID Controller
        intakePIDController = intakeSparkMax.getPIDController();
        intakePIDController.setP(Constants.BTConstants.PIDControllerConstants.P);
        intakePIDController.setD(Constants.BTConstants.PIDControllerConstants.D);
        intakePIDController.setI(Constants.BTConstants.PIDControllerConstants.I);
        intakePIDController.setFF(Constants.BTConstants.PIDControllerConstants.FF);
        intakePIDController.setOutputRange(Constants.BTConstants.PIDControllerConstants.MIN_OUTPUT, Constants.BTConstants.PIDControllerConstants.MAX_OUTPUT);
        intakePIDController.setSmartMotionMaxVelocity(Constants.BTConstants.PIDControllerConstants.IntakeSubsystemConstants.MAX_VELOCITY, 0);
        intakePIDController.setSmartMotionMinOutputVelocity(Constants.BTConstants.PIDControllerConstants.IntakeSubsystemConstants.MIN_VELOCITY, 0);
        intakePIDController.setSmartMotionMaxAccel(Constants.BTConstants.PIDControllerConstants.IntakeSubsystemConstants.MAX_ACCELERATION, 0);
        intakePIDController.setSmartMotionAllowedClosedLoopError(Constants.BTConstants.PIDControllerConstants.ALLOWED_ERROR, 0);


    }

    // Runs Intake At Max Speed
    public void RunAtMaxSpeed(){

        intakePIDController.setReference(5000, CANSparkMax.ControlType.kVelocity);

    }

    public void ReverseRun(){

        //intakePIDController.setReference(-11000, CANSparkMax.ControlType.kVelocity);
        intakeSparkMax.set(-1);

    }

    public void RunMotorAtSpeed(double inputSpeed){

        intakePIDController.setReference(inputSpeed, CANSparkMax.ControlType.kVelocity);

    }


    // Stops the Motor
    public void StopMotor(){

        intakePIDController.setReference(0, CANSparkMax.ControlType.kVelocity);

    }

    // Tells you when the limit switch is activated
    public boolean limitSwitchActivated(){

        return limitSwitch.isPressed();

    }

    public void periodic(){

        SmartDashboard.putBoolean("Note is In: ", limitSwitchActivated());
        SmartDashboard.putString("Intake Toggle: ", intakeToggleString);
    }

    public void intakeCommand(boolean activate, boolean outtake) {

        // This activates the different methods based on the toggle and when the limit switch is off
        //intakePosition = intakePos;
        if (!(limitSwitchActivated())){
            if (activate){
                RunAtMaxSpeed();
            }
            else if (outtake){
                ReverseRun();
            }
            else{
                StopMotor();
            }
        }

        // This runs when the limit switch is activated.
        else{

            if (outtake){
                ReverseRun();
            }
            else{
                StopMotor();
            }

        }


    }



}


