package frc.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkPIDController;
import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.SparkAbsoluteEncoder.Type;

public class BTAngleChangingSubsystem extends SubsystemBase{
    

    private CANSparkMax angleChangerSparkMax;
    private AbsoluteEncoder angleChangerEncoder;
    private SparkPIDController angleChangerPIDController;

    public BTAngleChangingSubsystem(){

        angleChangerSparkMax = new CANSparkMax(Constants.BTConstants.CANIdConstants.ANGLE_CHANGER_CAN_ID, MotorType.kBrushless);
        angleChangerSparkMax.restoreFactoryDefaults();

        angleChangerEncoder = angleChangerSparkMax.getAbsoluteEncoder(Type.kDutyCycle);
        angleChangerEncoder.setPositionConversionFactor(Constants.BTConstants.AngleChangingConstants.POSITION_CONVERSION_FACTOR);
        angleChangerEncoder.setVelocityConversionFactor(Constants.BTConstants.AngleChangingConstants.VELOCITY_CONVERSION_FACTOR);

        angleChangerPIDController = angleChangerSparkMax.getPIDController();

    }

    public void setAngle(int desiredAngle){



    }

    public void periodic(){

        SmartDashboard.putNumber("Angle Changer Position: ", angleChangerEncoder.getPosition());

    }





}
