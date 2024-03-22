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

  private void SetMotorPID() {
    TalonUtils.TalonPIDControl(m_intake, m_intake.getPosition().getValue());
  }

  public void IntakeForward() {
    TalonUtils.TalonVelocityControl(m_intake, 0.3);
  }

  public void IntakeMax() {
    TalonUtils.TalonVelocityControl(m_intake, 1);
  }

  public void IntakeOff() {
    TalonUtils.TalonVelocityControl(m_intake, 0);
  }

  public void IntakePIDOff() {
    SetMotorPID();
  }

  public void IntakeReverse() {
    TalonUtils.TalonVelocityControl(m_intake, -0.2);
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
