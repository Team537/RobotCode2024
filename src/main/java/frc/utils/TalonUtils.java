// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.utils;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj.motorcontrol.Talon;

/*
TalonFX commands that arent arm specific
*/
public class TalonUtils {
    /*
    Applys motion magic constants for the Arm 
    */
    private static void ApplyArmMotionMagicSlot(TalonFX m_talon, double targetPos) {
        double velocity = TargetDirVelo(m_talon,targetPos);
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
        
        m_talon.getConfigurator().apply(talonFXConfigs);
        
    }

    private static void ApplyPIDSlot(TalonFX m_talon) {
        var slot0Configs = new Slot0Configs();
        slot0Configs.kP = 1;
        slot0Configs.kI = 1;
        slot0Configs.kD = .01;

        m_talon.getConfigurator().apply(slot0Configs);
        var talonFXConfigs = new TalonFXConfiguration();
    }

    public static void ApplyRunMotorSlot(TalonFX m_talon) {
        var slot0Configs = new Slot0Configs();
        slot0Configs.kS = 0.05; // V to overcome static friction
        slot0Configs.kV = 0.12; // 1 rps = 0.12V output
        slot0Configs.kP = 0.11; // An error of 1 rps results in 0.11 V output

        m_talon.getConfigurator().apply(slot0Configs);
    }

    /*
    Directional calculator to calculate velocity 
    */
    private static double TargetDirVelo(TalonFX m_talon, double targetPos) {
        double currentPos = m_talon.getPosition().getValue();
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
    
    public static void TalonArmMotionMagicControl(TalonFX m_talon, double pos) {
        ApplyArmMotionMagicSlot(m_talon, pos);
        MotionMagicVoltage m_request = new MotionMagicVoltage(pos).withSlot(1);
        m_talon.setControl(m_request);
    }

    public static void TalonPIDControl(TalonFX m_talon, double pos) {
        ApplyPIDSlot(m_talon);
        // pos is the desired location of the falcon in rotations
        final PositionVoltage m_request = new PositionVoltage(pos).withSlot(0).withEnableFOC(true);
        m_talon.setControl(m_request);
    }

    public static void TalonVelocityControl(TalonFX m_talon, double percent) {
    //.withVelocity( input is RPS, 6380 is max Falcon rpm which converts to 106 and 1/3 rps)
    //.withFeedForward(input is V to overcome gravity)
    ApplyRunMotorSlot(m_talon);

    final VelocityVoltage m_request = new VelocityVoltage(0).withSlot(0);

    m_talon.setControl(m_request.withVelocity(percent*106.33));
//   }
    }
}
