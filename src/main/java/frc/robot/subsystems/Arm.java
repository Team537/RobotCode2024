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

public class Arm extends SubsystemBase {

  public static TalonFX m_arm1 = new TalonFX(ArmConstants.ARM1);
  public static TalonFX m_arm2 = new TalonFX(ArmConstants.ARM2);
  public static DutyCycleEncoder m_encoder = new DutyCycleEncoder(ArmConstants.ENCODER_DIO);

  final Follower m_follower = new Follower(ArmConstants.ARM2,false);

  double driveGyroYaw = 0;  // arm pigeon code, not used

  // just inits these variables, targetPos relys on the armgetpos so it doesnt move the arm to pos zero on teleop init
  public static double FalconArmTarget = m_arm1.getPosition().getValue();
  public double ChaseTarget = m_encoder.getAbsolutePosition();


  /** Creates a new Arm. */
  public Arm() {
    SmartDashboard.putNumber("CHASE CALC TARGET", 0);
  }

  private void TalonfxMotionMagicSlots(double targetPos) {
    double velocity = getTargetDir(targetPos);
    var talonFXConfigs = new TalonFXConfiguration();

    // set slot 0 gains
    var slot1Configs = talonFXConfigs.Slot1;
    slot1Configs.kV = 0.13; // A velocity target of 1 rps results in [var] V output
    // slot1Configs.kA = 0.04; // An acceleration of 1 rps/s requires [var] V output
    slot1Configs.kP = 4; // A position error of 2.5 rotations results in 12 V output  4
    slot1Configs.kI = 0; // no output for integrated error
    slot1Configs.kD = 0.1; // A velocity error of 1 rps results in 0.1 V output         0.5
    
    //configs slot1Configs.kg to be variable (arm cosine, with high gravity being at high)
    // talonFXConfigs.withSlot1(slot1Configs.withGravityType(GravityTypeValue.Arm_Cosine));

    
    // set Motion Magic settings
    var motionMagicConfigs = talonFXConfigs.MotionMagic;
    motionMagicConfigs.MotionMagicCruiseVelocity = velocity; // Target cruise velocity of 80 rps
    motionMagicConfigs.MotionMagicAcceleration = 40; // Target acceleration of 160 rps/s (0.5 seconds)
    motionMagicConfigs.MotionMagicJerk = 100; // Target jerk of 1600 rps/s/s (0.1 seconds)

    m_arm1.getConfigurator().apply(talonFXConfigs);
    m_arm2.getConfigurator().apply(talonFXConfigs);

  }

  private double getTargetDir(double targetPos) {
    double currentPos = m_arm2.getPosition().getValue();
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

  private MotionMagicVoltage TalonfxMotionMagic(double pos) {
    TalonfxMotionMagicSlots(pos);
    MotionMagicVoltage m_request = new MotionMagicVoltage(pos).withSlot(1);
    return m_request;
  }

  private void SetMotorsMotionMagic(double pos) {
    m_arm1.setControl(m_follower);
    m_arm2.setControl(TalonfxMotionMagic(pos));
  }

  public void ArmSubwoofer() { 
    SetMotorsMotionMagic(-7);
    FalconArmTarget = -7;
    // SetMotorsMotionMagic(findTargetDist(-7));
  }

  public void ArmIntake() {
    SetMotorsMotionMagic(0);
    FalconArmTarget = 0;
    // SetMotorsMotionMagic(findTargetDist(0));
  }
  
  public void ArmAmp() {
    SetMotorsMotionMagic(-55);
    FalconArmTarget = -55;
    // SetMotorsMotionMagic(findTargetDist(-55));
  }

  public void ArmMid() {
    SetMotorsMotionMagic(-23);
    FalconArmTarget = -23;
    // SetMotorsMotionMagic(findTargetDist(-23));
  }

  public void ArmMotionMagicStop() {
    SetMotorsMotionMagic(FalconArmTarget);
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
    ChaseTarget = 0.5;
    encoderChase(ChaseTarget);
  }

  public void ChaseSet0() {
    ChaseTarget = 0;
    encoderChase(ChaseTarget);
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
    SmartDashboard.putNumber("ABSOLUTE POS OFF", m_encoder.getAbsolutePosition()-ArmConstants.ENCODER_OFFSET);

    SmartDashboard.putNumber("CHASE TARGET", ChaseTarget);

    SmartDashboard.putNumber("Falcon Arm Target", FalconArmTarget);
    SmartDashboard.putNumber("ARM POS 1", m_arm1.getPosition().getValue());
    SmartDashboard.putNumber("ARM POS 2", m_arm2.getPosition().getValue());

    // This method will be called once per scheduler run
  }
}
