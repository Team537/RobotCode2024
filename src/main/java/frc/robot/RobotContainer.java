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
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
// import frc.robot.subsystems.cameras.RobotVision;
import frc.robot.Constants.VisionConstants;
import frc.robot.commands.vision.ResetImuWithVisionCommand;
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
  private final Arm Arm = new Arm();
  private final Intake Intake = new Intake();
  private final Shooter Shooter = new Shooter();
    private final RobotVision robotVision = new RobotVision.Builder()
        .addPhotonVisionCamera(VisionConstants.COLOR_CAMERA_NAME, VisionConstants.BACK_CAMERA_OFFSET,
            VisionConstants.APRIL_TAG_PIPELINE)
        .build();
    private final DriveSubsystem driveSubsystem = new DriveSubsystem(true, robotVision::estimateRobotPose);

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
    () -> driveSubsystem.driveFromController(
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

  // Alternative Command Options
  private final RunCommand targetPositionCommand = new RunCommand(() -> driveSubsystem.driveToPosition(
        new Pose2d(-5 * driverController.getLeftX(),
        5 * driverController.getLeftY(),
        new Rotation2d(0))), 
        driveSubsystem);

   
  /**
    * The container for the robot. Contains subsystems, OI devices, and commands.
    */
    // SmartDashboard options
    private final SendableChooser<Command> controllerSelection = new SendableChooser<>();
    private final SendableChooser<AutonomousOption> autonomousSelection = new SendableChooser<>();

    /**
     * The container for the robot. Contains subsystems, OI devices, and commands.
     */
    public RobotContainer() {
      //Bumpers ------------------------------------------------

      leftBumper.onTrue(new ParallelCommandGroup( new StartEndCommand(Shooter::ShooterForward, Shooter::ShooterForward,Shooter), 
      new StartEndCommand(Intake::IntakeOff, Intake::IntakeMax, Intake).withTimeout(1)));

      leftBumper.onFalse(new ParallelCommandGroup( new StartEndCommand(Shooter::ShooterStop, Shooter::ShooterStop,Shooter), 
      new StartEndCommand(Intake::IntakeOff, Intake::IntakeOff, Intake)));


      rightBumper.toggleOnTrue(new ParallelCommandGroup(new StartEndCommand(Intake::IntakeForward, Intake::IntakePIDOff, Intake).until(()-> Intake.GetSwitchHit()),
      new StartEndCommand(Arm::ArmManualStop, Arm::ArmSubwoofer, Arm).until(()-> Intake.GetSwitchHit())));

      // rightBumper.onFalse(new StartEndCommand(Intake::IntakeOff, Intake::IntakeOff, Intake));

      //ABXY ---------------------------------------------------------

      aButton.onTrue(new StartEndCommand(Arm::ArmIntake, Arm::ArmMotionMagicStop, Arm));

      // aButton.onFalse(null);


      bButton.onTrue(new StartEndCommand(Arm::ArmSubwoofer, Arm::ArmMotionMagicStop, Arm));

      // bButton.onFalse(null);


      xButton.onTrue(new StartEndCommand(Arm::ArmMid, Arm::ArmMotionMagicStop, Arm));

      // xButton.onFalse(null);


      yButton.onTrue(new StartEndCommand(Arm::ArmAmp, Arm::ArmMotionMagicStop, Arm));

      // yButton.onFalse(null);


      //D-PAD ---------------------------------------------

      dPadUpButton.onTrue(new StartEndCommand(Arm::ChaseSet0, Arm::ChaseSet0, Arm));

      // dPadUpButton.onFalse(null);


      dPadDownButton.onTrue(new StartEndCommand(Arm::ChaseSet05, Arm::ChaseSet05, Arm));

      // dPadDownButton.onFalse(null);


      dPadLeftButton.onTrue(new StartEndCommand(Arm::ArmManualUp, Arm::ArmManualUp, Arm));

      dPadLeftButton.onFalse(new StartEndCommand(Arm::ArmManualStop, Arm::ArmManualStop, Arm));


      dPadRightButton.onTrue(new StartEndCommand(Arm::ArmManualDown, Arm::ArmManualDown, Arm));

      dPadRightButton.onFalse(new StartEndCommand(Arm::ArmManualStop, Arm::ArmManualStop, Arm));


      //Start and Back --------------------------------------------

      // Reset the IMU when the start button is pressed.
     startButton.onTrue(new InstantCommand(driveSubsystem::zeroHeading));
     
      //startButton.onFalse(null);


      backButton.onTrue(new ParallelCommandGroup( new StartEndCommand(Shooter::ShooterReverse, Shooter::ShooterStop,Shooter), 
      new StartEndCommand(Intake::IntakeReverse, Intake::IntakeOff, Intake)));

      backButton.onFalse(new ParallelCommandGroup( new StartEndCommand(Shooter::ShooterStop, Shooter::ShooterStop,Shooter), 
      new StartEndCommand(Intake::IntakeOff, Intake::IntakeOff, Intake)));
      // */      

      // Configure the button bindings
      configureButtonBindings();

    // Adjust the dafult command based on which controler the driver ants to use.
    if (SmartDashboard.getBoolean("useXBoxController", true)) {
      driveSubsystem.setDefaultCommand(xBoxControllerCommand);
    } else {
      driveSubsystem.setDefaultCommand(flightstickCommand);
    } 
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

        // Determines whether or not we want to run autonomous.
        SmartDashboard.putBoolean("Run Auto", false);
        SmartDashboard.putBoolean("Run Shoot Auto Alone", true);


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
        // If we want to run autonomous, then follow the trajectory. Otherwise don't run the auto.
        if (SmartDashboard.getBoolean("Run Auto", false)) {

             // Run path following command, then stop at the end.
            return new SequentialCommandGroup(
            // new InstantCommand(driveSubsystem::zeroHeading),
            new StartEndCommand(Arm::ArmSubwoofer, Arm::ArmMotionMagicStop, Arm),
            new RunCommand(Shooter::ShooterForward, Shooter).withTimeout(1),
         new ParallelCommandGroup(new RunCommand(Shooter::ShooterForward, Shooter), new RunCommand(Intake::IntakeMax, Intake)).withTimeout(1),
            new ParallelCommandGroup(new RunCommand(Shooter::ShooterStop, Shooter), new RunCommand(Intake::IntakeOff, Intake)).withTimeout(1), 
            autonomousCommand.andThen(() -> driveSubsystem.drive(0, 0, 0, 0, false, false)));
        } else if (SmartDashboard.getBoolean("Run Shoot Auto Alone", true)) {
            return new SequentialCommandGroup(
            new StartEndCommand(Arm::ArmSubwoofer, Arm::ArmMotionMagicStop, Arm),
            new RunCommand(Shooter::ShooterForward, Shooter).withTimeout(1),
         new ParallelCommandGroup(new RunCommand(Shooter::ShooterForward, Shooter), new RunCommand(Intake::IntakeMax, Intake)).withTimeout(1),
            new ParallelCommandGroup(new RunCommand(Shooter::ShooterStop, Shooter), new RunCommand(Intake::IntakeOff, Intake)).withTimeout(1));

        } 
        return null;
    }

    /**
     * Start making the robot follow the selected auto path.
     */
    private void auto() {
        driveSubsystem.followTrajectory();
    }
}