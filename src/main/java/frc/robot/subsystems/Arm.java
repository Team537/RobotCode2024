// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ArmConstants;

public class Arm extends SubsystemBase {

  TalonFX m_arm1 = new TalonFX(ArmConstants.ARM1);
  TalonFX m_arm2 = new TalonFX(ArmConstants.ARM2);
  
  /** Creates a new Arm. */
  public Arm() {
    var slot0Configs = new Slot0Configs();
    slot0Configs.kP = 1;
    slot0Configs.kI = 1;
    slot0Configs.kD = 1;

    m_arm1.getConfigurator().apply(slot0Configs);
    m_arm2.getConfigurator().apply(slot0Configs);
  }

  

  public void ArmIntake() {
    

    
    final PositionVoltage m_request = new PositionVoltage(0).withSlot(0);

    m_arm1.setControl(m_request.withPosition(0.1));
  }

  public void ArmShoot() {

  }
  
  public void ArmAmp() {

  }
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
