// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.motorcontrol.Talon;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.SparkAbsoluteEncoder.Type;
import com.revrobotics.SparkPIDController;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.RelativeEncoder;

import frc.robot.Constants.ModuleConstants;
import frc.utils.SwerveUtils;

public class MAXSwerveModule {
  private final TalonFX drivingTalonFX;
  private final CANSparkMax turningSparkMax;

  private final AbsoluteEncoder turningEncoder;

  private final SparkPIDController turningPidController;

  private double chassisAngularOffset = 0;
  private SwerveModuleState desiredState = new SwerveModuleState(0.0, new Rotation2d());

  /**
   * Constructs a MAXSwerveModule and configures the driving and turning motor,
   * encoder, and PID controller. This configuration is specific to the REV
   * MAXSwerve Module built with NEOs, SPARKS MAX, and a Through Bore
   * Encoder.
   */
  public MAXSwerveModule(int drivingCANId, int turningCANId, double angularOffset) {
    drivingTalonFX = new TalonFX(drivingCANId);
    turningSparkMax = new CANSparkMax(turningCANId, MotorType.kBrushless);

    // Factory reset, so we get the SPARKS MAX to a known state before configuring
    // them. This is useful in case a SPARK MAX is swapped out.

    //Talon Defaults
    drivingTalonFX.getConfigurator().apply(new TalonFXConfiguration());
    drivingTalonFX.getConfigurator().refresh(SwerveUtils.generateDriveMotorConfig());
    drivingTalonFX.getConfigurator().refresh(SwerveUtils.generateDriveOpenLoopRampConfigs());
    drivingTalonFX.getConfigurator().refresh(SwerveUtils.generateDriveClosedloopRampConfigs());

    turningSparkMax.restoreFactoryDefaults();

    // Setup encoders and PID controllers for the driving and turning SPARKS MAX.
    turningEncoder = turningSparkMax.getAbsoluteEncoder(Type.kDutyCycle);
    turningPidController = turningSparkMax.getPIDController();
    turningPidController.setFeedbackDevice(turningEncoder);

    // Apply position and velocity conversion factors for the driving encoder. The
    // native units for position and velocity are rotations and RPM, respectively,
    // but we want meters and meters per second to use with WPILib's swerve APIs.

    // Apply position and velocity conversion factors for the turning encoder. We
    // want these in radians and radians per second to use with WPILib's swerve
    // APIs.

    //talons dont have this so its converted in line. they also default to RPS not RPM
    //(around line 166, drivingTalon.setControl(setSpeed(MPS to RPS)))
    turningEncoder.setPositionConversionFactor(ModuleConstants.TURNING_ENCODER_POSITION_FACTOR);
    turningEncoder.setVelocityConversionFactor(ModuleConstants.TURNING_ENCODER_VELOCITY_FACTOR);

    // Invert the turning encoder, since the output shaft rotates in the opposite direction of
    // the steering motor in the MAXSwerve Module.
    turningEncoder.setInverted(ModuleConstants.TURNING_ENCODER_INVERTED);

    // Enable PID wrap around for the turning motor. This will allow the PID
    // controller to go through 0 to get to the setpoint i.e. going from 350 degrees
    // to 10 degrees will go through 0 rather than the other direction which is a
    // longer route.
    turningPidController.setPositionPIDWrappingEnabled(true);
    turningPidController.setPositionPIDWrappingMinInput(ModuleConstants.TURNING_ENCODER_POSITION_PID_MIN_INPUT);
    turningPidController.setPositionPIDWrappingMaxInput(ModuleConstants.TURNING_ENCODER_POSITION_PID_MAX_INPUT);

    // Set the PID gains for the driving motor. Note these are example gains, and you
    // may need to tunes them for your own robot!
    // Line 51-54
    var slot0Configs = new Slot0Configs();
    slot0Configs.kG = ModuleConstants.DRIVING_FF;
    slot0Configs.kP = ModuleConstants.DRIVING_KP;
    slot0Configs.kI = ModuleConstants.DRIVING_KI;
    slot0Configs.kD = ModuleConstants.DRIVING_KD;
    drivingTalonFX.getConfigurator().apply(slot0Configs);

    // Set the PID gains for the turning motor. Note these are example gains, and you
    // may need to tune them for your own robot!
    turningPidController.setP(ModuleConstants.TURNING_KP);
    turningPidController.setI(ModuleConstants.TURNING_KI);
    turningPidController.setD(ModuleConstants.TURNING_KD);
    turningPidController.setFF(ModuleConstants.TURNING_FF);
    turningPidController.setOutputRange(ModuleConstants.TURNING_MIN_OUTPUT,
        ModuleConstants.TURNING_MAX_OUTPUT);

    drivingTalonFX.setNeutralMode(ModuleConstants.DRIVE_IDLE);
    turningSparkMax.setIdleMode(ModuleConstants.TURNING_MOTOR_IDLE_MODE);

    CurrentLimitsConfigs m_current = new CurrentLimitsConfigs();
    m_current.SupplyCurrentLimit = ModuleConstants.DRIVING_MOTOR_CURRENT_LIMIT;
    m_current.StatorCurrentLimitEnable = true;
    turningSparkMax.setSmartCurrentLimit(ModuleConstants.TURNING_MOTOR_CURRENT_LIMIT);

    // Save the SPARK MAX configurations. If a SPARK MAX browns out during
    // operation, it will maintain the above configurations.
    drivingTalonFX.getConfigurator().apply(m_current);
    turningSparkMax.burnFlash();

    chassisAngularOffset = angularOffset;
    desiredState.angle = new Rotation2d(turningEncoder.getPosition());
    drivingTalonFX.setPosition(0);
  }

  /**
   * Returns the current state of the module.
   *
   * @return The current state of the module.
   */
  public SwerveModuleState getState() {
    // Apply chassis angular offset to the encoder position to get the position
    // relative to the chassis.
    return new SwerveModuleState(drivingTalonFX.getVelocity().getValue(),
        new Rotation2d(turningEncoder.getPosition() - chassisAngularOffset));
  }

  /**
   * Returns the current position of the module.
   *
   * @return The current position of the module.
   */
  public SwerveModulePosition getPosition() {
    // Apply chassis angular offset to the encoder position to get the position
    // relative to the chassis.
    return new SwerveModulePosition(
        drivingTalonFX.getPosition().getValue(),
        new Rotation2d(turningEncoder.getPosition() - chassisAngularOffset));
  }

  /**
   * Sets the desired state for the module.
   *
   * @param desiredState Desired state with speed and angle.
   */
  public void setDesiredState(SwerveModuleState state) {
    // Apply chassis angular offset to the desired state.
    SwerveModuleState correctedDesiredState = new SwerveModuleState();
    correctedDesiredState.speedMetersPerSecond = desiredState.speedMetersPerSecond;
    correctedDesiredState.angle = desiredState.angle.plus(Rotation2d.fromRadians(chassisAngularOffset));

    // Optimize the reference state to avoid spinning further than 90 degrees.
    SwerveModuleState optimizedDesiredState = SwerveModuleState.optimize(correctedDesiredState,
        new Rotation2d(turningEncoder.getPosition()));

    // Command driving and turning SPARKS MAX towards their respective setpoints.

                                                   // RPM to RPS
    drivingTalonFX.setControl(SetSpeed((optimizedDesiredState.speedMetersPerSecond)/(Math.PI*ModuleConstants.WHEEL_DIAMETER_METERS)));
    turningPidController.setReference(optimizedDesiredState.angle.getRadians(), CANSparkMax.ControlType.kPosition);

    desiredState = state;
  }

  private VelocityVoltage SetSpeed(double velocity) {
    final VelocityVoltage m_request = new VelocityVoltage(0).withSlot(0);
    return m_request.withVelocity(velocity);
  }
}