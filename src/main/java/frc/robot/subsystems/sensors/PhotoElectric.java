// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.sensors;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class PhotoElectric extends SubsystemBase {

  DigitalInput m_photoelectric = new DigitalInput(0);
  /** Creates a new PhotoElectric. */
  public PhotoElectric() {}
  
  public boolean GetSwitchHit() {
    return m_photoelectric.get();
  }

  @Override
  public void periodic() {
    SmartDashboard.putBoolean("Photoelectric Sensor Value", m_photoelectric.get());

    // This method will be called once per scheduler run
  }

 
}
