package frc.robot.subsystems;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.SparkAbsoluteEncoder.Type;

public class BTIntakeSubsytem extends SubsystemBase{

    // Constants (will eventually go into the Constants Class)
    private int intakeWheelCANID = 1;
    private int intakeRaiserCANID = 2;

    // Field Creation
    private final CANSparkMax intakeWheelSparkMax;
    private final CANSparkMax intakeRaiserSparkMax;

    private final AbsoluteEncoder intakeRaiserEncoder;

    // Constructor
    public BTIntakeSubsytem(){
        
        intakeWheelSparkMax = new CANSparkMax(intakeWheelCANID, MotorType.kBrushless);
        intakeRaiserSparkMax = new CANSparkMax(intakeWheelCANID, MotorType.kBrushless);
        intakeRaiserEncoder = intakeRaiserSparkMax.getAbsoluteEncoder(Type.kDutyCycle);

    }

}
