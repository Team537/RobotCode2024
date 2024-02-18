package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.Constants;
import frc.robot.subsystems.BTRaisingSubsystem;

public class RaiseIntakeCommand extends Command{

    BTRaisingSubsystem m_BTRaisingSubsystem;

    public RaiseIntakeCommand(BTRaisingSubsystem btRaisingSubsystem){

        m_BTRaisingSubsystem = btRaisingSubsystem;

    }

    public void initialize(){

    }

    public void execute(){

        m_BTRaisingSubsystem.goToReleasePosition();

    }

    public void end(boolean interrupted){

    }

    public boolean isFinished(){

        // Command is finished when either the intake reaches its position
        // Or when the controlling button isn't triggered
        return (m_BTRaisingSubsystem.intakePosition() == Constants.BTConstants.IntakePositions.releasePosition);

    }

}
