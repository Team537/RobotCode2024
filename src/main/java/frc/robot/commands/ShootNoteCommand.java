package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.subsystems.BTIntakeSubsytem;
import frc.robot.subsystems.BTOutakeSubsytem;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;



public class ShootNoteCommand extends Command{

    private final BTOutakeSubsytem m_BTOutakeSubsytem;
    private final BTIntakeSubsytem m_BTIntakeSubsytem;
    private final Timer newTimer;


    public ShootNoteCommand(BTIntakeSubsytem btIntakeSubsytem, BTOutakeSubsytem btOutakeSubsytem){

        m_BTIntakeSubsytem = btIntakeSubsytem;
        m_BTOutakeSubsytem = btOutakeSubsytem;
        newTimer = new Timer();

    }

    public void initialize(){
    
        m_BTIntakeSubsytem.ReverseRun();
        newTimer.delay(5);
        m_BTIntakeSubsytem.StopMotor();

    }

    public void execute(){

        m_BTOutakeSubsytem.RunAtMaxSpeed();

    }

    public void end(boolean interrupted){

        m_BTIntakeSubsytem.StopMotor();
        m_BTOutakeSubsytem.StopMotor();

    }

    public boolean isFinished(){

        return false;

    }
    

}