package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.subsystems.BTIntakeSubsytem;
import frc.robot.subsystems.BTOutakeSubsytem;
import edu.wpi.first.wpilibj.Timer;



public class ShootNoteCommand extends Command{

    private final BTOutakeSubsytem m_BTOutakeSubsytem;
    private final BTIntakeSubsytem m_BTIntakeSubsytem;
    private final Timer newTimer;
    private final JoystickButton controllingButton;


    public ShootNoteCommand(BTIntakeSubsytem btIntakeSubsytem, BTOutakeSubsytem btOutakeSubsytem, JoystickButton controlButton){

        m_BTIntakeSubsytem = btIntakeSubsytem;
        m_BTOutakeSubsytem = btOutakeSubsytem;
        newTimer = new Timer();

        //We take in the button controlling the command as a parameter, so that we can use its boolean
        //value in the isFinished method, to decide whether or not to stop the command
        controllingButton = controlButton;

    }

    public void initialize(){
    
        m_BTIntakeSubsytem.RunMotorAtSpeed(500);
        newTimer.delay(5);
        m_BTIntakeSubsytem.StopMotor();

    }

    public void execute(){

        m_BTOutakeSubsytem.RunAtMaxSpeed();

    }

    public void end(){

        m_BTIntakeSubsytem.StopMotor();
        m_BTOutakeSubsytem.StopMotor();

    }

    public boolean isFinished(){

        return !(controllingButton.getAsBoolean());

    }



    

}