// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;

import frc.robot.Constants.*;
import frc.utils.TalonUtils;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Intake extends SubsystemBase {

  TalonFX m_intake = new TalonFX(IntakeConstants.INTAKE);
  DigitalInput m_photoelectric = new DigitalInput(IntakeConstants.PHOTOELECTRIC_DIO);

  /** Creates a new Intake. */
  public Intake() {
    TalonUtils.ApplyRunMotorSlot(m_intake);
  }

  private VelocityVoltage SetSpeed(double velocity, double feedforward) {
    //.withVelocity( input is RPS, 6380 is max Falcon rpm which converts to 106 and 1/3 rps)
    //.withFeedForward(input is V to overcome gravity)

    final VelocityVoltage m_request = new VelocityVoltage(0).withSlot(0);

    return m_request.withVelocity(velocity).withFeedForward(feedforward);
  }

  private PositionVoltage TalonfxPIDControl(double pos) {
    TalonfxPIDSlots();
    // pos is the desired location of the falcon in rotations
    final PositionVoltage m_request = new PositionVoltage(pos).withSlot(0).withEnableFOC(true);
    return m_request;
  }
  private void TalonfxPIDSlots() {
    var slot0Configs = new Slot0Configs();
    slot0Configs.kP = 1;
    slot0Configs.kI = 1;
    slot0Configs.kD = .01;

    m_intake.getConfigurator().apply(slot0Configs);

    var talonFXConfigs = new TalonFXConfiguration();
  }

  public void IntakeForward() {
    m_intake.set(0.3);
  }

  public void IntakeAmp() {
    m_intake.set(0.4);
  }
  

  public void IntakeMax() {
    m_intake.set(1);
  }

  public void IntakeOff() {
    m_intake.set(0);
  }

  public void IntakePIDOff() {
    m_intake.setControl(TalonfxPIDControl(m_intake.getPosition().getValue()));
  }

  public void IntakeReverse() {
    m_intake.set(-0.2);
  }

  public boolean GetSwitchHit() {
    return m_photoelectric.get();
  }

  @Override
  public void periodic() {
    SmartDashboard.putBoolean("Photoelectric Sensor Value", m_photoelectric.get());

    // This method will be called once per scheduler run
  }
}
