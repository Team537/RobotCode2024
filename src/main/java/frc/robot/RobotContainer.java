// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.List;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController.Button;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.CameraConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.cameras.RobotVision;

/*
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {

    // The robot's subsystems
    private final DriveSubsystem driveSubsystem = new DriveSubsystem(true);
    private final RobotVision robotVision = new RobotVision.Builder()
        .addPhotonVisionCamera(CameraConstants.COLOR_CAMERA_NAME, CameraConstants.BACK_CAMERA_OFFSET, CameraConstants.OBJECT_DETECTION_PIPELINE)
        .build();
  private final Arm Arm = new Arm();
  private final Intake Intake = new Intake();
  private final Shooter Shooter = new Shooter();

   // The driver's controller
   private final XboxController driverController = new XboxController(OIConstants.DRIVER_CONTROLLER_PORT);
   private final Joystick flightStick = new Joystick(OIConstants.DRIVER_CONTROLLER_PORT);

   // Setup each button on each driverController
   JoystickButton aButton = new JoystickButton(driverController, Button.kA.value);
   JoystickButton bButton = new JoystickButton(driverController, Button.kB.value);
   JoystickButton yButton = new JoystickButton(driverController, Button.kY.value);
   JoystickButton xButton = new JoystickButton(driverController, Button.kX.value);
   JoystickButton startButton = new JoystickButton(driverController, Button.kStart.value);
   JoystickButton backButton = new JoystickButton(driverController, Button.kBack.value);
   JoystickButton leftBumper = new JoystickButton(driverController, Button.kLeftBumper.value);
   JoystickButton rightBumper = new JoystickButton(driverController, Button.kRightBumper.value);
   POVButton dPadUpButton = new POVButton(driverController, 0);
   POVButton dPadDownButton = new POVButton(driverController, 180);
   POVButton dPadLeftButton = new POVButton(driverController, 90);
   POVButton dPadRightButton = new POVButton(driverController, 270);

  // Controller commands
  private final RunCommand xBoxControllerCommand = new RunCommand(
    () -> driveSubsystem.drive(
        -MathUtil.applyDeadband(driverController.getLeftY(), OIConstants.DRIVE_DEADBAND),
        -MathUtil.applyDeadband(driverController.getLeftX(), OIConstants.DRIVE_DEADBAND),
        -MathUtil.applyDeadband(driverController.getRightX(), OIConstants.DRIVE_DEADBAND),
        -MathUtil.applyDeadband(driverController.getRightY(), OIConstants.DRIVE_DEADBAND),
        driverController.getRightTriggerAxis(),
        true, true),
        driveSubsystem);

  private final RunCommand flightstickCommand = new RunCommand(
      () -> driveSubsystem.drive(
          -MathUtil.applyDeadband(flightStick.getY(), OIConstants.DRIVE_DEADBAND),
          -MathUtil.applyDeadband(flightStick.getX(), OIConstants.DRIVE_DEADBAND),
          -MathUtil.applyDeadband(flightStick.getTwist(), OIConstants.DRIVE_DEADBAND),
          0,
          0,
          true, true),
          driveSubsystem);

  // SmartDashboard options
  private final SendableChooser<Command> controllerSelection = new SendableChooser<>();

  // Alternative Command Options
  private final RunCommand targetPositionCommand = new RunCommand(() -> driveSubsystem.position(
        new Pose2d(-5 * driverController.getLeftX(),
        5 * driverController.getLeftY(),
        new Rotation2d(0))), 
        driveSubsystem);
   
  /**
    * The container for the robot. Contains subsystems, OI devices, and commands.
    */
    public RobotContainer() {

      rightBumper.onTrue(new StartEndCommand(Intake::IntakeForward, Intake::IntakeOff, Intake));
      rightBumper.onFalse(new StartEndCommand(Intake::IntakeOff, Intake::IntakeOff, Intake));
    
      leftBumper.onTrue(new StartEndCommand(Shooter::ShooterForward, Shooter::ShooterStop,Shooter));
      leftBumper.onFalse(new StartEndCommand(Shooter::ShooterStop, Shooter::ShooterStop,Shooter));

      bButton.onTrue(new StartEndCommand(Intake::IntakeReverse, Intake::IntakeOff, Intake));
      bButton.onFalse(new StartEndCommand(Intake::IntakeOff, Intake::IntakeOff, Intake));

      yButton.onTrue(new StartEndCommand(Shooter::ShooterAmp, Shooter::ShooterStop,Shooter));
      yButton.onFalse(new StartEndCommand(Shooter::ShooterStop, Shooter::ShooterStop,Shooter));

      backButton.onTrue(new StartEndCommand(Shooter::ShooterReverse, Shooter::ShooterStop, Shooter));
      backButton.onFalse(new StartEndCommand(Shooter::ShooterStop, Shooter::ShooterStop, Shooter));

      startButton.onTrue(new StartEndCommand(Intake::IntakeMax, Intake::IntakeOff, Intake));
      startButton.onFalse(new StartEndCommand(Intake::IntakeOff, Intake::IntakeOff, Intake));

      // dPadUpButton.onTrue(new StartEndCommand(Arm::ArmShoot, Arm::ArmShoot, Arm));
      // dPadDownButton.onTrue(new StartEndCommand(Arm::ArmAmp, Arm::ArmAmp, Arm));

      dPadRightButton.onTrue(new StartEndCommand(Arm::ArmManual1, Arm::ArmManualStop, Arm));
      dPadLeftButton.onTrue(new StartEndCommand(Arm::ArmManual2, Arm::ArmManualStop, Arm));


      dPadDownButton.onFalse(new StartEndCommand(Arm::ArmManualStop, Arm::ArmManualStop, Arm));
      dPadUpButton.onFalse(new StartEndCommand(Arm::ArmManualStop, Arm::ArmManualStop, Arm));
      dPadRightButton.onFalse(new StartEndCommand(Arm::ArmManualStop, Arm::ArmManualStop, Arm));
      dPadLeftButton.onFalse(new StartEndCommand(Arm::ArmManualStop, Arm::ArmManualStop, Arm));

        // Configure the button bindings
        configureButtonBindings();


    // Adjust the dafult command based on which controler the driver ants to use.
    if (SmartDashboard.getBoolean("useXBoxController", true)) {
      driveSubsystem.setDefaultCommand(xBoxControllerCommand);
    } else {
      driveSubsystem.setDefaultCommand(flightstickCommand);
    }
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

        // Move the robot's wheels into an X to prevent movement.
        // starButton1.whileTrue(new RunCommand(
        //                 // () -> driveSubsystem.setX(),
        //                 // driveSubsystem));

        backButton.onTrue(new RunCommand(
                        () -> driveSubsystem.zeroHeading(),
                        driveSubsystem));
    }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {

    // Create config for trajectory
    TrajectoryConfig config = new TrajectoryConfig(
        AutoConstants.MAX_SPEED_METERS_PER_SECOND,
        AutoConstants.MAX_ACCELERATION_METERS_PER_SECOND_SQUARED)
        // Add kinematics to ensure max speed is actually obeyed
        .setKinematics(DriveConstants.DRIVE_KINEMATICS);

    // An example trajectory to follow. All units in meters.
    Trajectory exampleTrajectory = TrajectoryGenerator.generateTrajectory(
        // Start at the origin facing the +X direction
        new Pose2d(0, 0, new Rotation2d(0)),
        // Pass through these two interior waypoints, making an 's' curve path
        List.of(new Translation2d(1, 1), new Translation2d(2, -1)),
        // End 3 meters straight ahead of where we started, facing forward
        new Pose2d(3, 0, new Rotation2d(0)),
        config);

    var thetaController = new ProfiledPIDController(
        AutoConstants.THETA_CONTROLLER_KP, 0, 0, AutoConstants.kThetaControllerConstraints);
    thetaController.enableContinuousInput(-Math.PI, Math.PI);

    SwerveControllerCommand swerveControllerCommand = new SwerveControllerCommand(
        exampleTrajectory,
        driveSubsystem::getPose, // Functional interface to feed supplier
        DriveConstants.DRIVE_KINEMATICS,

        // Position controllers
        new PIDController(AutoConstants.X_CONTROLLER_KP, 0, 0),
        new PIDController(AutoConstants.Y_CONTROLLER_KP, 0, 0),
        thetaController,
        driveSubsystem::setModuleStates,
        driveSubsystem);

    // Reset odometry to the starting pose of the trajectory.
    driveSubsystem.resetOdometry(exampleTrajectory.getInitialPose());

    // Run path following command, then stop at the end.
    return swerveControllerCommand.andThen(() -> driveSubsystem.drive(0, 0, 0, 0, 0,false, false));
  }
}