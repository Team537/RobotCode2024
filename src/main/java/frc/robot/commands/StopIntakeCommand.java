package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.BTIntakeSubsytem;

public class StopIntakeCommand extends Command{

    BTIntakeSubsytem m_BTIntakeSubsytem;

    public StopIntakeCommand(BTIntakeSubsytem btIntakeSubsytem){

        m_BTIntakeSubsytem = btIntakeSubsytem;

    }

    public void initialize(){

    }

    public void execute(){

        m_BTIntakeSubsytem.StopMotor();

    }

    public void end(boolean interrupted){

        m_BTIntakeSubsytem.RunAtMaxSpeed();

    }

    public boolean isFinished(){

        return false;

    }

    

    
}
