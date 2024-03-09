// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ShooterConstants;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.controls.VelocityVoltage;

public class Shooter extends SubsystemBase {

  public TalonFX m_shooter = new TalonFX(ShooterConstants.SHOOTER);

  /** Creates a new Shooter. */
  public Shooter() {
    // put falcon defaults here
    var slot0Configs = new Slot0Configs();
    slot0Configs.kS = 0.05; // V to overcome static friction
    slot0Configs.kV = 0.12; // 1 rps = 0.12V output
    slot0Configs.kP = 0.11; // An error of 1 rps results in 0.11 V output

    m_shooter.getConfigurator().apply(slot0Configs);
  }

  private VelocityVoltage SetSpeed(double velocity, double feedforward) {
    //.withVelocity( input is RPS, 6380 is max Falcon rpm which converts to 106 and 1/3 rps)
    //.withFeedForward(input is V to overcome gravity)

    final VelocityVoltage m_request = new VelocityVoltage(0).withSlot(0);

    return m_request.withVelocity(velocity).withFeedForward(feedforward);
  }

  public void ShooterForward() {    
    m_shooter.setControl(SetSpeed(-100, 0));
  }

  public void ShooterStop() {
    m_shooter.set(0);
  }

  public void ShooterAmp() {
    m_shooter.set(-0.1);
  }

  public void ShooterReverse() {
    m_shooter.set(0.1);
  }

  public double getShooterPos() {
    return m_shooter.getPosition().getValue();
  }


  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
