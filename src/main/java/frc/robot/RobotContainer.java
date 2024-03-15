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
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import frc.robot.Constants.VisionConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.commands.vision.ResetImuWithVisionCommand;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.cameras.RobotVision;
import frc.utils.Autonomous.AutonomousOption;

/*
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {

    // The robot's subsystems
    private final RobotVision robotVision = new RobotVision.Builder()
        .addPhotonVisionCamera(VisionConstants.COLOR_CAMERA_NAME, VisionConstants.BACK_CAMERA_OFFSET,
            VisionConstants.APRIL_TAG_PIPELINE)
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
      * an alternative command used for testing PID Controllers
      */
     private final RunCommand driveToPosition = new RunCommand(
        () -> driveSubsystem.driveToPosition(new Pose2d(
                driverController.getLeftX(),
                driverController.getLeftY(),
                new Rotation2d(Math.atan2(
                        driverController.getRightY(),
                        driverController.getRightX()
                ))
        )),driveSubsystem
     );

    // SmartDashboard options
    private final SendableChooser<Command> controllerSelection = new SendableChooser<>();
    private final SendableChooser<AutonomousOption> autonomousSelection = new SendableChooser<>();

    /**
     * The container for the robot. Contains subsystems, OI devices, and commands.
     */
    public RobotContainer() {

        // Setup all the neccicery SmartDashboard elements
        setupDashboard();

        // Configure the button bindings
        configureButtonBindings();
    }

    /**
     * Configures the robot so that the controlls match what was configured on SMartDashboard. (e.g 
     * whether the robot is being controlled with a flightstick or an xbox controller).
     */
    public void configureDriverPrefferences() {

        // Get the drive command for the selected controller.(Note that the
        // getSelected() method returns the command assosiated with each option, which
        // is set above).
        driveSubsystem.setDefaultCommand(controllerSelection.getSelected());

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
        starButton.whileTrue(new RunCommand(
                () -> driveSubsystem.setX(),
                driveSubsystem));

        backButton.onTrue(new ResetImuWithVisionCommand(driveSubsystem, robotVision));
    }

    /**
     * This method contains the logic for everything relating to the set up of
     * SmartDashboard. Currently, this
     * covers things like auto and controller selection.
     */
    private void setupDashboard() {

        // Setup controller selection.
        controllerSelection.setDefaultOption("Xbox Controller", xBoxControllerCommand);
        controllerSelection.addOption("Flightstick", flightstickCommand);
        controllerSelection.addOption("Drive To Position Test", driveToPosition);

        // Setup autonomous selection.
        // Loop through all of the available auto options and add each of them as a
        // seperate autonomous option
        // in SmartDashboard.
        autonomousSelection.setDefaultOption("RED_1", AutonomousOption.RED_1);
        for (AutonomousOption autonomousOption : AutonomousOption.values()) {
            autonomousSelection.addOption(autonomousOption.toString(), autonomousOption);
        }

        // Add all of the configured SmartDashboard elrments to the GUI.
        SmartDashboard.putData("Controller Selection", controllerSelection);
        SmartDashboard.putData("Autonomous Selection", autonomousSelection);
    }

    /**
     * rests the default commands for the robot to the selected smart dashboard values
     */
    public void updateCommands() {
        driveSubsystem.setDefaultCommand(controllerSelection.getSelected());
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