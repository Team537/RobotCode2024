package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.BTRaisingSubsystem;

public class LowerIntakeCommand extends Command{

    BTRaisingSubsystem m_BTRaisingSubsystem;

    public LowerIntakeCommand(BTRaisingSubsystem btRaisingSubsystem){

        m_BTRaisingSubsystem = btRaisingSubsystem;

    }

    public void initialize(){

    }

    public void execute(){

        m_BTRaisingSubsystem.goToIntakePosition();
    }
    
    public void end(){

    }

    public boolean isFinished(){

        return m_BTRaisingSubsystem.intakePosition() == Constants.BTConstants.IntakePositions.intakePosition;

    }

}
