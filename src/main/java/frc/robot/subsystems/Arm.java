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

  public static TalonFX m_follower = new TalonFX(ArmConstants.ARM1);
  public static TalonFX m_leader = new TalonFX(ArmConstants.ARM2);
  public static DutyCycleEncoder m_encoder = new DutyCycleEncoder(ArmConstants.ENCODER_DIO);

  final Follower m_followControl = new Follower(ArmConstants.ARM2,false);
  
  // just inits these variables, targetPos relys on the armgetpos so it doesnt move the arm to pos zero on teleop init
  public static double MotionMagicTarget = m_follower.getPosition().getValue();
  public double EncoderTarget = m_encoder.getAbsolutePosition();


  /** Creates a new Arm. */
  public Arm() {
    SmartDashboard.putNumber("Encoder Calculated Target", 0);
  }

  private void SetMotorsMotionMagic(double pos) {
    m_follower.setControl(m_followControl);
    TalonUtils.TalonArmMotionMagicControl(m_leader, pos);
    MotionMagicTarget = pos;
  }

  private void SetMotorsPID(double pos) {
    m_follower.setControl(m_followControl);
    TalonUtils.TalonPIDControl(m_leader, pos);
  }

  private void SetMotorsVelocity(double percent) {
    TalonUtils.TalonVelocityControl(m_follower, percent);
    TalonUtils.TalonVelocityControl(m_leader, percent);
  }

  private void EncoderChase(double targetENC) {
    EncoderTarget = targetENC;
    double currentENC = m_encoder.getAbsolutePosition();
    double targetMotor = ((targetENC-currentENC)*ArmConstants.GEAR_RATIO)+(m_leader.getPosition().getValue());

    SmartDashboard.putNumber("Encoder Calculated Target", targetMotor);
    //sends the pos to the motors
    SetMotorsMotionMagic(targetMotor);
  }
  

  //Arm Motion Magic Relative Encoder
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


  //Arm Manual
  public void ArmManualDown() {
    SetMotorsVelocity(0.2);
  }

  public void ArmManualUp() {
    SetMotorsVelocity(-0.3);
  }

  public void ArmManualStop() {
    SetMotorsPID(m_leader.getPosition().getValue());
  } 


  //Arm Motion Magic Absolute Encoder
  public void ChaseSet05() {
    EncoderChase(0.5);
  }

  public void ChaseSet0() {
    EncoderChase(0);
  }

  //periodic (runs every 20 ms)
  @Override
  public void periodic() {
    SmartDashboard.putNumber("Encoder Pos", m_encoder.getAbsolutePosition());
    SmartDashboard.putNumber("Encoder Target", EncoderTarget);
    SmartDashboard.putNumber("Motion Magic Arm Target", MotionMagicTarget);
    SmartDashboard.putNumber("Follower Arm Positon", m_follower.getPosition().getValue());
    SmartDashboard.putNumber("Leader Arm Positon", m_leader.getPosition().getValue());

    // This method will be called once per scheduler run
  }
}
