// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ArmConstants;

public class Arm extends SubsystemBase {

  TalonFX m_arm1 = new TalonFX(ArmConstants.ARM1);
  TalonFX m_arm2 = new TalonFX(ArmConstants.ARM2);

  final Follower m_follower = new Follower(11,false);

  /** Creates a new Arm. */
  public Arm() {    
    var slot0Configs = new Slot0Configs();
    slot0Configs.kP = 1;
    slot0Configs.kI = 1;
    slot0Configs.kD = .01;

    m_arm1.getConfigurator().apply(slot0Configs);
    m_arm2.getConfigurator().apply(slot0Configs);
  }

  private PositionVoltage TalonfxPIDControl(double pos) {
    // pos is the desired location of the falcon in rotations
    final PositionVoltage m_request = new PositionVoltage(pos).withSlot(0).withEnableFOC(true);
    return m_request;
  }

  private void SetMotorsPID(double pos) {
    m_arm1.setControl(TalonfxPIDControl(pos));
    m_arm2.setControl(m_follower);
  }

  public void ArmSubwoofer() {
   SetMotorsPID(-20);
  }

  public void ArmIntake() {
    // SetMotorsPID(0);
  }
  
  public void ArmAmp() {
    SetMotorsPID(0);
  }

  public void ArmMid() {
    // SetMotorsPID(0);
  }

  public void ArmClimbUp() {
    // SetMotorsPID(0);
  }

  public void ArmClimbDown() {
    // SetMotorsPID(0);
  }

  public void ArmManualDown() {
    m_arm1.set(0.2);
    m_arm2.set(0.2);
  }
  public void ArmManualUp() {
    m_arm1.set(-0.3);
    m_arm2.set(-0.3);
  }
  public void ArmManualStop() {
    SetMotorsPID(m_arm1.getPosition().getValue());
  }
  @Override
  public void periodic() {
    SmartDashboard.putNumber("ARM POS", m_arm1.getPosition().getValue());

    // This method will be called once per scheduler run
  }
}
