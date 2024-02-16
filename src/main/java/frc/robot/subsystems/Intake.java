// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import frc.robot.Constants.*;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Intake extends SubsystemBase {

  CANSparkMax m_intake = new CANSparkMax(IntakeConstants.INTAKE, MotorType.kBrushless);

  /** Creates a new Intake. */
  public Intake() {
    //put spark defaults here
  }

  public void IntakeForward() {
    m_intake.set(0.1);
  }

  public void IntakeOff() {
    m_intake.set(0);
  }

  public void IntakeReverse() {
    m_intake.set(-0.1);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
