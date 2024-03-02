// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;

import frc.robot.Constants.*;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Intake extends SubsystemBase {

  TalonFX m_intake = new TalonFX(IntakeConstants.INTAKE);

  /** Creates a new Intake. */
  public Intake() {
    //put spark defaults here
  }

  public void IntakeForward() {
    m_intake.set(0.4);
  }

  public void IntakeMax() {
    m_intake.set(1);
  }

  public void IntakeOff() {
    m_intake.set(0);
  }

  public void IntakeReverse() {
    m_intake.set(-0.2);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
