package frc.robot.subsystems;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

public class BTOutakeSubsytem extends SubsystemBase{

    // Constants: will eventually be moved to constants class
    private int lmCANID = 2;
    private int rmCANID = 3;

    //Fields Declaration (To be changed to Talon FX but need the REV json library)
    private final CANSparkMax leftMotorSparkMax;
    private final CANSparkMax rightMotorSparkMax;

    public BTOutakeSubsytem(){

        leftMotorSparkMax = new CANSparkMax(3, MotorType.kBrushless);
        rightMotorSparkMax = new CANSparkMax(4, MotorType.kBrushless);

    }




    
}
