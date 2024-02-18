package frc.robot.commands;


import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.subsystems.BTIntakeSubsytem;

public class IntakeNoteCommand extends Command{

    private final BTIntakeSubsytem m_BTIntakeSubsytem;
    private final JoystickButton controllingButton;

    public IntakeNoteCommand(BTIntakeSubsytem btIntakeSubsytem, JoystickButton controlButton){

        m_BTIntakeSubsytem = btIntakeSubsytem;

        //We take in the button controlling the command as a parameter, so that we can use its boolean
        //value in the isFinished method
        controllingButton = controlButton;

    }

    public void initialize(){

    }

    public void execute(){

        m_BTIntakeSubsytem.RunMotorAtSpeed(1500);

    }

    public void end(){

        m_BTIntakeSubsytem.StopMotor();

    }

    public boolean isFinished(){

        return (m_BTIntakeSubsytem.limitSwitchActivated() || !(controllingButton.getAsBoolean()));

    }

}
