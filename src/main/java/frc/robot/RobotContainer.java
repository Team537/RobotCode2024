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
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
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
    private final RobotVision robotVision = new RobotVision.Builder()
            .addPhotonVisionCamera(CameraConstants.COLOR_CAMERA_NAME, CameraConstants.BACK_CAMERA_OFFSET,
                    CameraConstants.OBJECT_DETECTION_PIPELINE)
            .build();
    private final DriveSubsystem driveSubsystem = new DriveSubsystem(true, robotVision::estimateRobotPose);

    // The driver's controller
    private final XboxController driverController = new XboxController(OIConstants.DRIVER_CONTROLLER_PORT);
    private final Joystick flightStick = new Joystick(OIConstants.DRIVER_CONTROLLER_PORT);

    // Setup all possible controller inputs to make creating commands easier.
    JoystickButton starButton = new JoystickButton(driverController, Button.kStart.value);
    JoystickButton backButton = new JoystickButton(driverController, Button.kBack.value);
    JoystickButton rightStick = new JoystickButton(driverController, Button.kRightStick.value);
    JoystickButton leftStick = new JoystickButton(driverController, Button.kLeftStick.value);
    JoystickButton rightBumper = new JoystickButton(driverController, Button.kRightBumper.value);
    JoystickButton leftBumper = new JoystickButton(driverController, Button.kLeftBumper.value);
    JoystickButton aButton = new JoystickButton(driverController, Button.kA.value);
    JoystickButton bButton = new JoystickButton(driverController, Button.kB.value);
    JoystickButton yButton = new JoystickButton(driverController, Button.kY.value);
    JoystickButton xButton = new JoystickButton(driverController, Button.kX.value);
    POVButton dpadUpButton = new POVButton(driverController, 0);
    POVButton dpadDownButton = new POVButton(driverController, 180);
    POVButton dpadRightButton = new POVButton(driverController, 90);
    POVButton dpadLeftButton = new POVButton(driverController, 270);

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
                    -MathUtil.applyDeadband(flightStick.getY(), OIConstants.DRIVE_DEADBAND),
                    -MathUtil.applyDeadband(flightStick.getTwist(), OIConstants.DRIVE_DEADBAND),
                    0,
                    0,
                    true, true),
            driveSubsystem);

    // SmartDashboard options
    private final SendableChooser<Command> controllerSelection = new SendableChooser<>();

    /**
     * The container for the robot. Contains subsystems, OI devices, and commands.
     */
    public RobotContainer() {

        // Configure the button bindings
        configureButtonBindings();

        // Setup controller selection.
        controllerSelection.setDefaultOption("Xbox Controller", xBoxControllerCommand);
        controllerSelection.addOption("Flightstick", flightstickCommand);

        // Create the selection object in SmartDashboard (If it doesn't already exist)
        SmartDashboard.putData("Controller Selection", controllerSelection);

        // Get the drive command for the selected controller.(Note that the
        // getSelected() method returns
        // the command assosiated with each option, which is set above).
        driveSubsystem.setDefaultCommand(controllerSelection.getSelected());
    }


    private void configureButtonBindings() {

        // Move the robot's wheels into an X to prevent movement.
        starButton.whileTrue(new RunCommand(
                () -> driveSubsystem.setX(),
                driveSubsystem));

        backButton.onTrue(new RunCommand(
                () -> driveSubsystem.zeroHeading(),
                driveSubsystem));
    }

    /**
     * Takes a photongraph using all of the cameras.
     */
    public void snapshot() {
        robotVision.snapshotAll();
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
        return swerveControllerCommand.andThen(() -> driveSubsystem.drive(0, 0, 0, 0, 0, false, false));
    }
}