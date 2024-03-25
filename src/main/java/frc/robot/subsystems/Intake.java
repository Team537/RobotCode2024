// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;

import frc.robot.Constants.*;
import frc.utils.TalonUtils;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Intake extends SubsystemBase {

  //intake Talon
  private TalonFX m_intake = new TalonFX(IntakeConstants.INTAKE);

  //photoelectric sensor, with DIO port
  private DigitalInput m_photoelectric = new DigitalInput(IntakeConstants.PHOTOELECTRIC_DIO);

  /* Creates a new Intake.
  Runs on robot Init
  */
  public Intake() {
    //Applys motor config defaults for the Intake
    TalonUtils.ApplyRunMotorSlot(m_intake);
  }

  //Local PID set command. The intake only sets PID to its current position, so it needs no parameters
  private void SetMotorPID() {
    TalonUtils.TalonPIDControl(m_intake, m_intake.getPosition().getValue());
  }


  //Intake max speed, runs with shooting motor
  public void IntakeMax() {
    TalonUtils.TalonVelocityControl(m_intake, 1);
  }

  //Intake Pickup notes
  public void IntakeForward() {
    TalonUtils.TalonVelocityControl(m_intake, 0.3);
  }

  //turns the intake off, used specifically after shooting notes
  public void IntakeOff() {
    TalonUtils.TalonVelocityControl(m_intake, 0);
  }

  //backup, reverses intake. runs in parrallel with shooter reverse to reposition notes
  public void IntakeReverse() {
    TalonUtils.TalonVelocityControl(m_intake, -0.2);
  }


  //this command sets the motors to go to its current PID position
  //it is used when intaking notes, and will set to the position when the photoelectric sensor goes off. 
  public void IntakePIDOff() {
    SetMotorPID();
  }

  //used to detect when the photoelectric sensor sees a note
  public boolean GetSwitchHit() {
    return m_photoelectric.get();
  }

  @Override
  public void periodic() {
    SmartDashboard.putBoolean("Photoelectric Sensor Value", m_photoelectric.get());
    

    // This method will be called once per scheduler run (every 20ms)
  }
}
