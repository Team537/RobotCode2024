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

import frc.robot.Constants.FalconMotorConstants;
import frc.robot.Constants.TalonMotionMagicConstants;
import frc.robot.Constants.TalonPIDConstants;
import frc.robot.Constants.TalonVelocityConstants;

/*
Broad talonfx commands, will config motion magic, PID, and velocity
simplifies subsystems
*/
public class TalonUtils {
    
    //Applys motion magic constants for the Arm 
    
    private static void ApplyArmMotionMagicSlot(TalonFX m_talon, double targetPos) {
        //gets the velocity based on the direction of the arm. the arm goes slower going down
        double velocity = TargetDirVelo(m_talon,targetPos);

        //makes a new config
        var talonFXConfigs = new TalonFXConfiguration();
        
        // create new slot1 configs
        // slot0 is used by PID only
        var slot1Configs = talonFXConfigs.Slot1;

        //feed forward for gravity, velocity, and gravity
        slot1Configs.kV = TalonMotionMagicConstants.KV; // A velocity target of 1 rps results in [var] V output
        slot1Configs.kA = TalonMotionMagicConstants.KA; // An acceleration of 1 rps/s requires [var] V output
        slot1Configs.kG = TalonMotionMagicConstants.KG; //Gravity FeedForward

        //PID
        slot1Configs.kP = TalonMotionMagicConstants.KP; // A position error of 2.5 rotations results in 12 V output
        slot1Configs.kI = TalonMotionMagicConstants.KI; // no output for integrated error
        slot1Configs.kD = TalonMotionMagicConstants.KD; // A velocity error of 1 rps results in 0.1 V output
        //KD value notes - 3/21/2024 ********************************************
        // when kd was set to 0.5, motor shock violently. keep motor at or below 0.1
        //***************************************************
        
        //configs slot1Configs.kg to be variable (arm cosine, with high gravity being at high)
        //untested, dont know what it does
        // talonFXConfigs.withSlot1(slot1Configs.withGravityType(GravityTypeValue.Arm_Cosine));
        
            
        // set Motion Magic settings
        var motionMagicConfigs = talonFXConfigs.MotionMagic;
        motionMagicConfigs.MotionMagicCruiseVelocity = velocity; // Target cruise velocity of [var] rps
        motionMagicConfigs.MotionMagicAcceleration = TalonMotionMagicConstants.ACCELERATION; // Target acceleration of [var] rps/s (0.5 seconds)
        motionMagicConfigs.MotionMagicJerk = TalonMotionMagicConstants.JERK; // Target jerk of [var] rps/s/s (0.1 seconds)
        
        //applys config to the motor
        m_talon.getConfigurator().apply(talonFXConfigs);
        
    }

    //applys PID configs
    private static void ApplyPIDSlot(TalonFX m_talon) {
        //gets new slot0 config
        var slot0Configs = new Slot0Configs();
        //PID
        slot0Configs.kP = TalonPIDConstants.KP;
        slot0Configs.kI = TalonPIDConstants.KI;
        slot0Configs.kD = TalonPIDConstants.KD;

        //Applys config to motor
        m_talon.getConfigurator().apply(slot0Configs);
    }

    //applys configs for Velocity Mode
    public static void ApplyRunMotorSlot(TalonFX m_talon) {

        //creates new config
        var slot0Configs = new Slot0Configs();
        //feedforward[static], feedforward[velocity], p gain
        slot0Configs.kS = TalonVelocityConstants.KS; // V to overcome static friction
        slot0Configs.kV = TalonVelocityConstants.KV; // 1 rps = 0.12V output
        slot0Configs.kP = TalonVelocityConstants.KP; // An error of 1 rps results in 0.11 V output

        //applys configs
        m_talon.getConfigurator().apply(slot0Configs);
    }

    
    //Directional calculator to calculate velocity 
    private static double TargetDirVelo(TalonFX m_talon, double targetPos) {
        double currentPos = m_talon.getPosition().getValue();
        double velocity = 0;
        if (targetPos < currentPos) {
            //up
            velocity = TalonMotionMagicConstants.CRUISE_VELOCITY_UP;
        } else if (targetPos > currentPos) {
            //down
            velocity = TalonMotionMagicConstants.CRUISE_VELOCITY_DOWN;
        } else if (targetPos == currentPos) {
            //not moving? idk trust me bro
            velocity = 0;
        }
        return velocity;
    }
    
    // gets motor and position for motion magic control
    public static void TalonArmMotionMagicControl(TalonFX m_talon, double pos) {
        //applys the motion magic slot. pos is used to calculate velo for directional [TargetDirVelo()]
        ApplyArmMotionMagicSlot(m_talon, pos);

        //creates a new motion magic request with target pos of [pos] and with the slot 1 configs
        MotionMagicVoltage m_request = new MotionMagicVoltage(pos).withSlot(1);

        //tells motor to move to the position
        m_talon.setControl(m_request);
    }

    // gets motor and position for pid control
    public static void TalonPIDControl(TalonFX m_talon, double pos) {
        // applys PID slot configs
        ApplyPIDSlot(m_talon);

        // pos is the desired location of the falcon in rotations
        //creates a new PID position request with pos [pos] with the PID slot 0. FOC makes better command chaining
        final PositionVoltage m_request = new PositionVoltage(pos).withSlot(0).withEnableFOC(true);

        //tells motor to move to the position
        m_talon.setControl(m_request);
    }

    //gets motor and percent for velocity control
    //percent is the percent of the max falcon rps [106.33 rps = 6380 rpm]
    public static void TalonVelocityControl(TalonFX m_talon, double percent) {
        //some other configs that can be used
        //.withVelocity( input is RPS, 6380 is max Falcon rpm which converts to 106 and 1/3 rps)
        //.withFeedForward(input is V to overcome gravity)

        //applys slot 0 configs for velocity
        ApplyRunMotorSlot(m_talon);

        //if the user is dumb and trys to send the falcon faster than it should, it sets the value back
        if (percent > 1) {
            percent = 1;
        } else if (percent < -1) {
            percent = -1;
        }

        //creates new velocity voltage request with the desired velocity and slot 0 configs
        final VelocityVoltage m_request = new VelocityVoltage(FalconMotorConstants.FREE_SPEED_RPS*percent).withSlot(0);

        m_talon.setControl(m_request);
    }
}
