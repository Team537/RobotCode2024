// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ShooterConstants;
import frc.utils.TalonUtils;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.controls.VelocityVoltage;

public class Shooter extends SubsystemBase {

  public TalonFX m_shooter = new TalonFX(ShooterConstants.SHOOTER);

  /** Creates a new Shooter. */
  public Shooter() {
    TalonUtils.ApplyRunMotorSlot(m_shooter);

    //this makes positive values send the shooter forward(shoot note)
    m_shooter.setInverted(true);
  }

  private VelocityVoltage SetSpeed(double velocity, double feedforward) {
    //.withVelocity( input is RPS, 6380 is max Falcon rpm which converts to 106 and 1/3 rps)
    //.withFeedForward(input is V to overcome gravity)

    final VelocityVoltage m_request = new VelocityVoltage(0).withSlot(0);

    return m_request.withVelocity(velocity).withFeedForward(feedforward);
  }

  public void ShooterForward() {    
    TalonUtils.TalonVelocityControl(m_shooter, 1);
  }

  public void ShooterStop() {
    m_shooter.set(0);
  }

  public void ShooterAmp() {
    TalonUtils.TalonVelocityControl(m_shooter, 0.1);
  }

  public void ShooterReverse() {
    TalonUtils.TalonVelocityControl(m_shooter, -0.1);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
