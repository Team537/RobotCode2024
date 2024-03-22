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
import com.ctre.phoenix6.signals.GravityTypeValue;

import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ArmConstants;
import frc.utils.TalonUtils;
import frc.utils.TalonUtils.*;

public class Arm extends SubsystemBase {

  public static TalonFX m_arm1 = new TalonFX(ArmConstants.ARM1);
  public static TalonFX m_arm2 = new TalonFX(ArmConstants.ARM2);
  public static DutyCycleEncoder m_encoder = new DutyCycleEncoder(ArmConstants.ENCODER_DIO);

  final Follower m_follower = new Follower(ArmConstants.ARM2,false);
  
  // just inits these variables, targetPos relys on the armgetpos so it doesnt move the arm to pos zero on teleop init
  public static double MotionMagicTarget = m_arm1.getPosition().getValue();
  public double EncoderTarget = m_encoder.getAbsolutePosition();


  /** Creates a new Arm. */
  public Arm() {
    SmartDashboard.putNumber("CHASE CALC TARGET", 0);
  }

  private void SetMotorsMotionMagic(double pos) {
    m_arm1.setControl(m_follower);
    TalonUtils.TalonArmMotionMagic(m_arm2, pos);
    MotionMagicTarget = pos;
  }

  public void ArmSubwoofer() { 
    SetMotorsMotionMagic(-7);
  }

  public void ArmIntake() {
    SetMotorsMotionMagic(0);
  }
  
  public void ArmAmp() {
    SetMotorsMotionMagic(-55);
  }

  public void ArmMid() {
    SetMotorsMotionMagic(-23);
  }

  public void ArmMotionMagicStop() {
    SetMotorsMotionMagic(MotionMagicTarget);
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
    SetMotorsMotionMagic(m_arm2.getPosition().getValue());
  } 

  public void ChaseSet05() {
    EncoderTarget = 0.5;
    encoderChase(EncoderTarget);
  }

  public void ChaseSet0() {
    EncoderTarget = 0;
    encoderChase(EncoderTarget);
  }

  private void encoderChase(double targetENC) {
    double currentENC = m_encoder.getAbsolutePosition();
    double targetMotor = ((targetENC-currentENC)*ArmConstants.GEAR_RATIO)+(m_arm2.getPosition().getValue());

    SmartDashboard.putNumber("CHASE CALC TARGET", targetMotor);
    //sends the pos to the motors
    SetMotorsMotionMagic(targetMotor);
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("ABSOLUTE POS", m_encoder.getAbsolutePosition());
    SmartDashboard.putNumber("Encoder Target", EncoderTarget);
    SmartDashboard.putNumber("Motion Magic Arm Target", MotionMagicTarget);
    SmartDashboard.putNumber("ARM POS 1", m_arm1.getPosition().getValue());
    SmartDashboard.putNumber("ARM POS 2", m_arm2.getPosition().getValue());

    // This method will be called once per scheduler run
  }
}
