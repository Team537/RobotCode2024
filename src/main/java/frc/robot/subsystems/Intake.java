// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.function.BooleanSupplier;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;

import frc.robot.Constants.*;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Intake extends SubsystemBase {

  TalonFX m_intake = new TalonFX(IntakeConstants.INTAKE);

  /** Creates a new Intake. */
  public Intake() {
    var slot0Configs = new Slot0Configs();
    slot0Configs.kS = 0.05; // V to overcome static friction
    slot0Configs.kV = 0.12; // 1 rps = 0.12V output
    slot0Configs.kP = 0.11; // An error of 1 rps results in 0.11 V output

    m_intake.getConfigurator().apply(slot0Configs);
  }

  private VelocityVoltage SetSpeed(double velocity, double feedforward) {
    //.withVelocity( input is RPS, 6380 is max Falcon rpm which converts to 106 and 1/3 rps)
    //.withFeedForward(input is V to overcome gravity)

    final VelocityVoltage m_request = new VelocityVoltage(0).withSlot(0);

    return m_request.withVelocity(velocity).withFeedForward(feedforward);
  }

  public void IntakeForward() {
    m_intake.set(0.4);
  }

  public boolean IntakeMax() {
    System.out.println("intake max");
    m_intake.set(1);
    return true;
  }

  public void IntakeOff() {
    m_intake.set(0);
    System.out.println("RAN END");
  }

  public void IntakeReverse() {
    m_intake.set(-0.2);
  }
  public void IntakeOnEND() {
    System.out.println("RAN END");
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
