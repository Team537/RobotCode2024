package frc.robot.commands;


import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.subsystems.BTIntakeSubsytem;

public class IntakeNoteCommand extends Command{

    private final BTIntakeSubsytem m_BTIntakeSubsytem;

    public IntakeNoteCommand(BTIntakeSubsytem btIntakeSubsytem){

        m_BTIntakeSubsytem = btIntakeSubsytem;

    }

    public void initialize(){

    }

    public void execute(){

        m_BTIntakeSubsytem.RunMotorAtSpeed();

    }

    public void end(boolean interuppted){

        m_BTIntakeSubsytem.StopMotor();

    }

    public boolean isFinished(){

        return m_BTIntakeSubsytem.limitSwitchActivated();

    }

}
