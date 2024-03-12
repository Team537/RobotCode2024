// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.Pigeon2;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ArmConstants;

public class Arm extends SubsystemBase {

  TalonFX m_arm1 = new TalonFX(ArmConstants.ARM1);
  TalonFX m_arm2 = new TalonFX(ArmConstants.ARM2);
  
  Pigeon2 m_pigeon = new Pigeon2(ArmConstants.PIGEON);
  
  double targetPos = m_arm1.getPosition().getValue();

  final Follower m_follower = new Follower(11,false);

  /** Creates a new Arm. */
  public Arm() {    

  }

  private double ConvertAngleToRot() {
    double rotations = (m_pigeon.getAngle()/360)*200;
    return rotations;
  }
    private double ConvertRotToAngle() {
    double angle = (m_arm1.getPosition().getValue()/200)*360;
    return angle;
  }

  private void TalonfxMotionMagicSlots(double targetPos) {

    double velocity = getTargetDir(targetPos);
    var talonFXConfigs = new TalonFXConfiguration();

    // set slot 0 gains
    var slot1Configs = talonFXConfigs.Slot1;
    // slot0Configs.kS = 0.25; // Add 0.25 V output to overcome static friction
    slot1Configs.kV = 0.12; // A velocity target of 1 rps results in 0.12 V output
    slot1Configs.kA = 0.04; // An acceleration of 1 rps/s requires 0.01 V output
    slot1Configs.kP = 2; // A position error of 2.5 rotations results in 12 V output
    slot1Configs.kI = 0; // no output for integrated error
    slot1Configs.kD = 0; // A velocity error of 1 rps results in 0.1 V output

    
    // set Motion Magic settings
    var motionMagicConfigs = talonFXConfigs.MotionMagic;
    motionMagicConfigs.MotionMagicCruiseVelocity = velocity; // Target cruise velocity of 80 rps
    motionMagicConfigs.MotionMagicAcceleration = 40; // Target acceleration of 160 rps/s (0.5 seconds)
    motionMagicConfigs.MotionMagicJerk = 100; // Target jerk of 1600 rps/s/s (0.1 seconds)

    m_arm1.getConfigurator().apply(talonFXConfigs);
    m_arm2.getConfigurator().apply(talonFXConfigs);

  }
  private void TalonfxPIDSlots() {
    var slot0Configs = new Slot0Configs();
    slot0Configs.kP = 1;
    slot0Configs.kI = 1;
    slot0Configs.kD = .01;

    m_arm1.getConfigurator().apply(slot0Configs);
    m_arm2.getConfigurator().apply(slot0Configs);

    var talonFXConfigs = new TalonFXConfiguration();
  }

  private double getTargetDir(double targetPos) {
    double currentPos = m_arm1.getPosition().getValue();
    double velocity = 0;
    if (targetPos < currentPos) {
      //up
      velocity = 45;
    } else if (targetPos > currentPos) {
      velocity = 25;
    } else if (targetPos == currentPos) {
      velocity = 0;
    }
    return velocity;
  }

  private PositionVoltage TalonfxPIDControl(double pos) {
    TalonfxPIDSlots();
    // pos is the desired location of the falcon in rotations
    final PositionVoltage m_request = new PositionVoltage(pos).withSlot(0).withEnableFOC(true);
    return m_request;
  }

  private MotionMagicVoltage TalonfxMotionMagic(double pos) {
    TalonfxMotionMagicSlots(pos);
    MotionMagicVoltage m_request = new MotionMagicVoltage(pos).withSlot(1);
    return m_request;
  }

  private void SetMotorsPID(double pos) {
    m_arm1.setControl(TalonfxPIDControl(pos));
    m_arm2.setControl(m_follower);
  }

  private void SetMotorsMotionMagic(double pos) {
    m_arm1.setControl(TalonfxMotionMagic(pos));
    m_arm2.setControl(m_follower);
  }

  public void ArmSubwoofer() { 
    // -7 on one sprocket
    SetMotorsMotionMagic(-9);
    targetPos = -9;
  }

  public void ArmIntake() {
    SetMotorsMotionMagic(0);
    targetPos = 0;
  }
  
  public void ArmAmp() {
    SetMotorsMotionMagic(-55);
    targetPos = -50;
  }

  public void ArmMid() {
    SetMotorsMotionMagic(-23.6);
  }

  public void ArmPIDStop() {
    SetMotorsPID(targetPos);
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
  
  public boolean targetPid() {
    if ((targetPos - 0.2) < m_arm1.getPosition().getValue() && m_arm1.getPosition().getValue() < targetPos+0.2) {
      // return true;
      return true; 
    } else {
      return false;
    }
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("arm angle", m_pigeon.getAngle());
    SmartDashboard.putBoolean("withinPosRange", targetPid());
    SmartDashboard.putNumber("targetpos", targetPos);
    SmartDashboard.putNumber("ARM POS", m_arm1.getPosition().getValue());
    

    // This method will be called once per scheduler run
  }
}
