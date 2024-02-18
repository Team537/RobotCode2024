package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.BTOutakeSubsytem;

public class StopOuttakeCommand extends Command{

    BTOutakeSubsytem m_BTOutakeSubsytem;

    public StopOuttakeCommand(BTOutakeSubsytem btOutakeSubsytem){

        m_BTOutakeSubsytem = btOutakeSubsytem;

    }

    public void initialize(){

    }

    public void execute(){

        m_BTOutakeSubsytem.StopMotor();

    }

    public void end(boolean interrupted){

        m_BTOutakeSubsytem.RunAtMaxSpeed();

    }

    public boolean isFinished(){
        return false;
    }



}
