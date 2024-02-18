package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
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
    
    public void end(boolean interrupted){

    }

    public boolean isFinished(){

        // Is finished when intake either reaches the right position or the controlling button 
        // isn't triggered
        return (m_BTRaisingSubsystem.intakePosition() == Constants.BTConstants.IntakePositions.intakePosition);

    }

}
