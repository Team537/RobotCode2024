// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.net.PortForwarder;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.VisionConstants;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Shooter;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  private int timer = 0;
  private Command autonomousCommand;
  private RobotContainer robotContainer;
  private final Timer snapshotTimer = new Timer(); // Used to take photographs after a set period of time.

  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  @Override
  public void robotInit() {

    // Instantiate our RobotContainer.  This will perform all our button bindings, and put our
    // autonomous chooser on the dashboard.
    robotContainer = new RobotContainer();

    // Start the timer
    snapshotTimer.start();

    // Make it possible to view USB webcams connected to the RoboRio
    CameraServer.startAutomaticCapture();
    
    // Make it possible to view the photonvision dashboard over the internet
    PortForwarder.add(5800, "photonvision.local", 5800);
    timer = 0;

    
  }

  /**
   * This function is called every 20 ms, no matter the mode. Use this for items like diagnostics
   * that you want ran during disabled, autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before LiveWindow and
   * SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
    // Runs the Scheduler.  This is responsible for polling buttons, adding newly-scheduled
    // commands, running already-scheduled commands, removing finished or interrupted commands,
    // and running subsystem periodic() methods.  This must be called from the robot's periodic
    // block in order for anything in the Command-based framework to work.
    if (timer <= 15000) {
      timer += 20;
      System.out.println(timer);
      final MotorOutputConfigs m_coastConfig = new MotorOutputConfigs();
      m_coastConfig.NeutralMode = NeutralModeValue.Coast;
      Arm.m_arm2.getConfigurator().apply(m_coastConfig);
      Arm.m_arm1.getConfigurator().apply(m_coastConfig);
    } else if (false) {
      final MotorOutputConfigs m_brakeConfig = new MotorOutputConfigs();
      m_brakeConfig.NeutralMode = NeutralModeValue.Brake;
      Arm.m_arm2.getConfigurator().apply(m_brakeConfig);
      Arm.m_arm1.getConfigurator().apply(m_brakeConfig);
    }
    CommandScheduler.getInstance().run();
  }

  /** This function is called once each time the robot enters Disabled mode. */
  @Override
  public void disabledInit() {
    MotorOutputConfigs m_configs = new MotorOutputConfigs();
    m_configs.NeutralMode = NeutralModeValue.Brake;
    Arm.m_arm2.getConfigurator().apply(m_configs);
    Arm.m_arm1.getConfigurator().apply(m_configs);
  }

  @Override
  public void disabledPeriodic() {

  }

  /** This autonomous runs the autonomous command selected by your {@link RobotContainer} class. */
  @Override
  public void autonomousInit() {
    autonomousCommand = robotContainer.getAutonomousCommand();

    /*
     * String autoSelected = SmartDashboard.getString("Auto Selector",
     * "Default"); switch(autoSelected) { case "My Auto": autonomousCommand
     * = new MyAutoCommand(); break; case "Default Auto": default:
     * autonomousCommand = new ExampleCommand(); break; }
     */

    // schedule the autonomous command (example)
    MotorOutputConfigs m_configs = new MotorOutputConfigs();
    m_configs.NeutralMode = NeutralModeValue.Brake;
    Arm.m_arm2.getConfigurator().apply(m_configs);
    Arm.m_arm1.getConfigurator().apply(m_configs);
    if (autonomousCommand != null) {
      autonomousCommand.schedule();
    }
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {

     // Take a screenshot every 250ms.
     if (snapshotTimer.get() >= VisionConstants.SNAPSHOT_RATE) {
      snapshotTimer.reset();
      // robotContainer.snapshot();
    }
  }

  @Override
  public void teleopInit() {

    // Update the robot's settings so that they match what was configured on SmartDashboard.
    robotContainer.configureDriverPrefferences();

    // This makes sure that the autonomous stops running when
    // teleop starts running. If you want the autonomous to
    // continue until interrupted by another command, remove
    // this line or comment it out.
    robotContainer.updateCommands();
    if (autonomousCommand != null) {
      autonomousCommand.cancel();
    }
  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {

     // Take a screenshot every 250ms.
     if (snapshotTimer.get() >= VisionConstants.SNAPSHOT_RATE) {
      snapshotTimer.reset();
      // robotContainer.snapshot();
    }

    // System.out.println("Absolute POS OFFSET: " + (Arm.m_encoder.getAbsolutePosition()-ArmConstants.ENCODER_OFFSET));
    // System.out.println("TARGET: " + Arm.targetPos);
    // System.out.println("ARM POS2: " + Arm.m_arm2.getPosition().getValue());
    // System.out.println("ARM POS1: " + Arm.m_arm1.getPosition().getValue());

  }

  @Override
  public void testInit() {
    // Cancels all running commands at the start of test mode.
    CommandScheduler.getInstance().cancelAll();
  }

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {}
}
