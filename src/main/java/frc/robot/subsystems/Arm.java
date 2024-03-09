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
import frc.robot.Constants.FieldConstants;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;

public class Arm extends SubsystemBase {

  TalonFX m_arm1 = new TalonFX(ArmConstants.ARM1);
  TalonFX m_arm2 = new TalonFX(ArmConstants.ARM2);

  private double targetVal = 0;

  final Follower m_follower = new Follower(11,false);

  /** Creates a new Arm. */
  public Arm() {
    // m_arm2.setInverted(true);
    
    var slot0Configs = new Slot0Configs();
    // slot0Configs.kV = 0.5;
    // slot0Configs.kS = 0.5;
    slot0Configs.kP = 1;
    slot0Configs.kI = 1;
    slot0Configs.kD = .01;

    m_arm1.getConfigurator().apply(slot0Configs);
    m_arm2.getConfigurator().apply(slot0Configs);
  }

  

  public void ArmShoot() {
    

    
    final PositionVoltage m_request = new PositionVoltage(-20).withSlot(0).withEnableFOC(true);

    m_arm1.setControl(m_request);
    m_arm2.setControl(m_follower);
  }

  public void ArmIntake() {

  }
  
  public void ArmAmp() {

    final PositionVoltage m_request = new PositionVoltage(0).withSlot(0).withEnableFOC(true);

    m_arm1.setControl(m_request);
    m_arm2.setControl(m_follower);
  }

  public void ArmManual1() {
    m_arm1.set(0.2);
    m_arm2.set(0.2);
  }
  public void ArmManual2() {
    m_arm1.set(-0.3);
    m_arm2.set(-0.3);
  }
  public void ArmManualStop() {
    double pos = m_arm1.getPosition().getValue();
    final PositionVoltage m_request = new PositionVoltage(pos).withSlot(0).withEnableFOC(true);


    m_arm1.setControl(m_request);
    m_arm2.setControl(m_follower);
    System.out.println("RAN END ARM");

  }

  /**
   * sets the pitch to target the speaker
   * @param robotPose the current pose of the robot
   */
  public void targetSpeaker(Pose2d robotPose) {

    //get the distance from the robot to the speaker and subtract the offset of the arm (it is 6.75 inches closer)
    double distance = -ArmConstants.ARM_OFFSET + Math.sqrt( Math.pow(robotPose.getX() - FieldConstants.SPEAKER_POSE.getX(),2) + Math.pow(robotPose.getY() - FieldConstants.SPEAKER_POSE.getY(),2) );
    double angle = Math.atan2(FieldConstants.SPEAKER_HEIGHT - ArmConstants.ARM_HEIGHT,distance);
    double target = angle * (100 / Math.PI);
    
    final PositionVoltage m_request = new PositionVoltage(target).withSlot(0).withEnableFOC(true);
    m_arm1.setControl(m_request);
    m_arm2.setControl(m_follower);

    //makes the estimated target position a class variable so it can be sent to smart dashboard
    targetVal = target;

  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("ARM POS", m_arm1.getPosition().getValue());
    SmartDashboard.putNumber("TARGET POS", targetVal);

    // This method will be called once per scheduler run
  }
}
