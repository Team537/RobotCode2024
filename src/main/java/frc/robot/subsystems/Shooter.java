// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.ShooterConstants;
import com.ctre.phoenix6.controls.Follower;

public class Shooter extends SubsystemBase {

  TalonFX m_shooter = new TalonFX(ShooterConstants.SHOOTER);
  TalonFX m_shooter2 = new TalonFX(ArmConstants.ARM2);

  final Follower m_follower = new Follower(10, false);
  // m_shooter2.setControl(m_follower);

  /** Creates a new Shooter. */
  public Shooter() {
    // put falcon defaults here
  }

  public void ShooterForward() {
    m_shooter.set(0.15);
    m_shooter2.setControl(m_follower);
    // m_shooter2.set(0.3);

  }

  public void ShooterStop() {
    m_shooter.set(0);
    m_shooter2.setControl(m_follower);
    // m_shooter2.set(0);
  }

  public void ShooterReverse() {
    m_shooter.set(-0.1);
  }


  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
