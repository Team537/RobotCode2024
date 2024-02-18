// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController.Button;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController.Button;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.commands.IntakeNoteCommand;
import frc.robot.commands.LowerIntakeCommand;
import frc.robot.commands.RaiseIntakeCommand;
import frc.robot.commands.ShootNoteCommand;
import frc.robot.commands.StopIntakeCommand;
import frc.robot.commands.StopOuttakeCommand;
import frc.robot.subsystems.BTIntakeSubsytem;
import frc.robot.subsystems.BTOutakeSubsytem;
import frc.robot.subsystems.BTRaisingSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.LimelightVision;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import java.util.List;

import javax.xml.crypto.dsig.spec.HMACParameterSpec;

/*
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems
  private final DriveSubsystem driveSubsystem = new DriveSubsystem();
  private final LimelightVision LimelightVision = new LimelightVision();

    //**Black Team's Subsystems
  private final BTIntakeSubsytem intakeSubsystem = new BTIntakeSubsytem();
  private final BTOutakeSubsytem outakeSubsystem = new BTOutakeSubsytem();
  private final BTRaisingSubsystem raisingSubsystem = new BTRaisingSubsystem();


    // The driver's controller
    private final XboxController driverController = new XboxController(OIConstants.DRIVER_CONTROLLER_PORT);
    private final Joystick flightStick = new Joystick(OIConstants.DRIVER_CONTROLLER_PORT);

    JoystickButton aButton = new JoystickButton(m_driverController,Button.kA.value);
    JoystickButton bButton = new JoystickButton(m_driverController, Button.kB.value);
    JoystickButton xButton = new JoystickButton(m_driverController, Button.kX.value);
    JoystickButton yButton = new JoystickButton(m_driverController, Button.kY.value);
    JoystickButton leftBumper = new JoystickButton(m_driverController, Button.kLeftBumper.value);


    private final RunCommand intakeCommand = new RunCommand(
      () -> intakeSubsystem.intakeCommand(
        m_driverController.getAButton(),
        m_driverController.getLeftBumper()
      ),intakeSubsystem
    );

    private final RunCommand outakeCommand = new RunCommand(
      () -> outakeSubsystem.outakeCommand(
        m_driverController.getYButton()
      ),outakeSubsystem
    );

    private final RunCommand raisingCommand = new RunCommand(
      () -> raisingSubsystem.raisingCommand(
        m_driverController.getBButton(),
        m_driverController.getXButton()
      ),raisingSubsystem
    );

    JoystickButton aButton = new JoystickButton(m_driverController,Button.kA.value);
    JoystickButton bButton = new JoystickButton(m_driverController, Button.kB.value);
    JoystickButton xButton = new JoystickButton(m_driverController, Button.kX.value);
    JoystickButton yButton = new JoystickButton(m_driverController, Button.kY.value);





    // Controller commands
    private final RunCommand xBoxControllerCommand = new RunCommand(
            () -> driveSubsystem.driveFromController(
                    -MathUtil.applyDeadband(driverController.getLeftY(), OIConstants.DRIVE_DEADBAND),
                    -MathUtil.applyDeadband(driverController.getLeftX(), OIConstants.DRIVE_DEADBAND),
                    -MathUtil.applyDeadband(driverController.getRightX(), OIConstants.DRIVE_DEADBAND),
                    -MathUtil.applyDeadband(driverController.getRightY(), OIConstants.DRIVE_DEADBAND),
                    driverController.getRightTriggerAxis(),
                    true, true),
            driveSubsystem);

    private final RunCommand flightstickCommand = new RunCommand(
            () -> driveSubsystem.driveFromController(
                    -MathUtil.applyDeadband(flightStick.getY(), OIConstants.DRIVE_DEADBAND),
                    -MathUtil.applyDeadband(flightStick.getY(), OIConstants.DRIVE_DEADBAND),
                    -MathUtil.applyDeadband(flightStick.getTwist(), OIConstants.DRIVE_DEADBAND),
                    0,
                    0,
                    true, true),
            driveSubsystem);

          
  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {

    // Configure the button bindings
    configureButtonBindings();
    

    // Adjust the dafult command based on which controler the driver ants to use.
    if (SmartDashboard.getBoolean("useXBoxController", true)) {
      driveSubsystem.setDefaultCommand(xBoxControllerCommand);
    } else {
      driveSubsystem.setDefaultCommand(flightstickCommand);
    }

    intakeSubsystem.setDefaultCommand(intakeCommand);
    outakeSubsystem.setDefaultCommand(outakeCommand);
    raisingSubsystem.setDefaultCommand(raisingCommand);

  }

  /**
   * Use this method to define your button->command mappings. Buttons can be
   * created by
   * instantiating a {@link edu.wpi.first.wpilibj.GenericHID} or one of its
   * subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then calling
   * passing it to a
   * {@link JoystickButton}.
   */
  private void configureButtonBindings() {
    new JoystickButton(m_driverController, Button.kRightBumper.value)
        .whileTrue(new RunCommand(
            () -> driveSubsystem.setX(),
            driveSubsystem));

    // Button Bindings for Black Team Controls

  
  }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {
        
        // Get the selected auto
        AutonomousOption selectedAuto = autonomousSelection.getSelected();
        
        // Start making the robot follow a trajectory
        RunCommand autonomousCommand =  new RunCommand(
                () -> auto()
        );

        // Configure the robot's settings so that it will be optimized for the selected command.
        driveSubsystem.setAutonomous(selectedAuto);

        // Run path following command, then stop at the end.
        return autonomousCommand.andThen(() -> driveSubsystem.drive(0, 0, 0, 0, false, false));
    }

    /**
     * Start making the robot follow the selected auto path.
     */
    private void auto() {
        driveSubsystem.followTrajectory();
    }
}