// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ShooterConstants;
import frc.utils.TalonUtils;


public class Shooter extends SubsystemBase {

  //shooter talon
  private TalonFX m_shooter = new TalonFX(ShooterConstants.SHOOTER);

  /** Creates a new Shooter.
  runs on robot init
  */
  public Shooter() {
    //apply default values
    TalonUtils.ApplyRunMotorSlot(m_shooter);

    //this makes positive values send the shooter forward(shoot note)
    m_shooter.setInverted(true);
  }

  //shoots note
  public void ShooterForward() {    
    TalonUtils.TalonVelocityControl(m_shooter, 1);
  }

  //stops shooter
  public void ShooterStop() {
    m_shooter.set(0);
  }

  //backup shooter reverse to move notes within intake
  public void ShooterReverse() {
    TalonUtils.TalonVelocityControl(m_shooter, -0.1);
  }

  //amp specific, tyler request
  public void ShooterAmp() {
    TalonUtils.TalonVelocityControl(m_shooter, 0.75);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run(every 20 ms)
  }
}
