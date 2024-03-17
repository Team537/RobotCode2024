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

import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ArmConstants;

public class Arm extends SubsystemBase {

  public static TalonFX m_arm1 = new TalonFX(ArmConstants.ARM1);
  public static TalonFX m_arm2 = new TalonFX(ArmConstants.ARM2);
  public static DutyCycleEncoder m_encoder = new DutyCycleEncoder(ArmConstants.ENCODER_DIO);
  public static Pigeon2 m_armPigeon = new Pigeon2(ArmConstants.PIGEON);

  final Follower m_follower = new Follower(ArmConstants.ARM1,false);

  double driveGyroYaw = 0;  // arm pigeon code, not used

  // just inits these variables, targetPos relys on the armgetpos so it doesnt move the arm to pos zero on teleop init
  double targetPos = m_arm1.getPosition().getValue();
  double TargetVar = m_arm1.getPosition().getValue();


  /** Creates a new Arm. */
  public Arm() {
  }

  /* Pigeon on arm code
  private double ConvertAngleToRot() {
    double rotations = (m_pigeon.getAngle()/360)*200;
    return rotations;
  }
    private double ConvertRotToAngle() {
    double angle = (m_arm1.getPosition().getValue()/200)*360;
    return angle;
  }*/

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
      velocity = 60;
    } else if (targetPos > currentPos) {
      velocity = 40;
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
    SetMotorsMotionMagic(-7);
    targetPos = -7;
    TargetVar = findTargetDist(-7);

  }

  public void ArmIntake() {
    SetMotorsMotionMagic(0);
    targetPos = 0;
    TargetVar = findTargetDist(0);

  }
  
  public void ArmAmp() {
    SetMotorsMotionMagic(-55);
    targetPos = -55;
    TargetVar = findTargetDist(-55);

  }

  public void ArmMid() {
    SetMotorsMotionMagic(-23);
    targetPos = -23;
    TargetVar = findTargetDist(-23);

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
    if ((targetPos - 0.2) < m_arm1.getPosition().getValue() && m_arm1.getPosition().getValue() < targetPos + 0.2) {
      // if the motor value is within 0.2[Deadband] rot of the desired end location, enable the PID loop
      // and override the motionmagic trapezoidal loop. the PID loop is stronger at keeping the 
      // arm in position than the motion magic loop. 
      return true; 
    } else {
      return false;
    }
  }

  private double findTargetDist(double posIn) {
    double currentPos = motorRnd(m_arm1.getPosition().getValue()); //-55
    double currentEncoderPos = encoderRnd2Dec(m_encoder.getAbsolutePosition()-ArmConstants.ENCODER_OFFSET); //-55
                            //-23
    double distToTarget = (posIn - currentEncoderPos) + currentPos;

    // targetPos = distToTarget;
    
    return distToTarget;
  }

  private double encoderRnd2Dec(double num) {
    
    if (num < 0) {                  //mulltiply by 100 because of falcon gearbox (make encoder same ratio as falcons)
      return -1*Math.round(Math.abs(num*100));
    } else {
      return Math.round(Math.abs(num*100));
    }
  }

    private double motorRnd(double num) {
    if (num < 0) {
      return -1*Math.round(Math.abs(num));
    } else {
      return Math.round(Math.abs(num));
    }
  }

  @Override
  public void periodic() {

    SmartDashboard.putNumber("ABSOLUTE POS RND", encoderRnd2Dec(m_encoder.getAbsolutePosition()));
    SmartDashboard.putNumber("ABSOLUTE POS", m_encoder.getAbsolutePosition());

    SmartDashboard.putNumber("ABSOLUTE POS OFF RND", encoderRnd2Dec(m_encoder.getAbsolutePosition()-ArmConstants.ENCODER_OFFSET));
    SmartDashboard.putNumber("ABSOLUTE POS OFF", m_encoder.getAbsolutePosition()-ArmConstants.ENCODER_OFFSET);
    SmartDashboard.putNumber("TARGET", TargetVar);

    SmartDashboard.putNumber("ArmPitch", m_armPigeon.getPitch().getValue());
    SmartDashboard.putNumber("ArmYaw", m_armPigeon.getYaw().getValue());
    SmartDashboard.putBoolean("withinPosRange", targetPid());
    SmartDashboard.putNumber("targetpos", targetPos);
    SmartDashboard.putNumber("ARM POS", m_arm1.getPosition().getValue());
    

    // This method will be called once per scheduler run
  }
}
