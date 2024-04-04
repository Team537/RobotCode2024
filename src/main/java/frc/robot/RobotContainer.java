// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.List;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController.Button;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import frc.robot.Constants.VisionConstants;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.FieldConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.commands.FollowTrajectoryCommand;
import frc.robot.commands.vision.ResetImuWithVisionCommand;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.cameras.RobotVision;
import frc.utils.Autonomous.AutonomousOption;
import frc.utils.Autonomous.Alliance;


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
        .addPhotonVisionCamera(VisionConstants.ARDUCAM_OV2311_USB_CAMERA_NAME, VisionConstants.ARDUCAM_OV9281_OFFSET,
            VisionConstants.APRIL_TAG_PIPELINE)
        .addPhotonVisionCamera(VisionConstants.USB_2M_GS_CAMERA_NAME, VisionConstants.USB_2M_GS_CAMERA_OFFSET,
            VisionConstants.APRIL_TAG_PIPELINE)
        .build();
    private final DriveSubsystem driveSubsystem = new DriveSubsystem(true, robotVision::estimateRobotPose);

    // The driver's controller
    private final XboxController driverController = new XboxController(OIConstants.DRIVER_CONTROLLER_PORT);
    private final Joystick flightStick = new Joystick(OIConstants.DRIVER_CONTROLLER_PORT);

    // Setup all possible controller inputs to make creating commands easier.
    JoystickButton startButton = new JoystickButton(driverController, Button.kStart.value);
    JoystickButton backButton = new JoystickButton(driverController, Button.kBack.value);
    JoystickButton rightStick = new JoystickButton(driverController, Button.kRightStick.value);
    JoystickButton leftStick = new JoystickButton(driverController, Button.kLeftStick.value);
    JoystickButton rightBumper = new JoystickButton(driverController, Button.kRightBumper.value);
    JoystickButton leftBumper = new JoystickButton(driverController, Button.kLeftBumper.value);
    JoystickButton aButton = new JoystickButton(driverController, Button.kA.value);
    JoystickButton bButton = new JoystickButton(driverController, Button.kB.value);
    JoystickButton yButton = new JoystickButton(driverController, Button.kY.value);
    JoystickButton xButton = new JoystickButton(driverController, Button.kX.value);
    POVButton dPadUpButton = new POVButton(driverController, 0);
    POVButton dPadDownButton = new POVButton(driverController, 180);
    POVButton dPadRightButton = new POVButton(driverController, 90);
    POVButton dPadLeftButton = new POVButton(driverController, 270);
    

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
    private final SendableChooser<Alliance> teamAlliance = new SendableChooser<>();




    /**
     * The container for the robot. Contains subsystems, OI devices, and commands
     */
    public RobotContainer() {
    // Setup all the necessary SmartDashboard elements
    setupDashboard();

    //triggers -----------------------------------------------
    if (driverController.getLeftTriggerAxis() >= 0.8) {
        new ParallelCommandGroup( new StartEndCommand(Shooter::ShooterAmp, Shooter::ShooterAmp,Shooter), 
        new StartEndCommand(Intake::IntakeAmp, Intake::IntakeAmp, Intake));
    } else {
        new ParallelCommandGroup( new StartEndCommand(Shooter::ShooterStop, Shooter::ShooterStop,Shooter), 
        new StartEndCommand(Intake::IntakeStop, Intake::IntakeStop, Intake));
    }
        
    //Bumpers ------------------------------------------------

    leftBumper.onTrue(new ParallelCommandGroup( new StartEndCommand(Shooter::ShooterForward, Shooter::ShooterForward,Shooter), 
        new StartEndCommand(Intake::IntakeStop, Intake::IntakeMax, Intake).withTimeout(0.75)));

    leftBumper.onFalse(new ParallelCommandGroup( new StartEndCommand(Shooter::ShooterStop, Shooter::ShooterStop,Shooter), 
        new StartEndCommand(Intake::IntakeStop, Intake::IntakeStop, Intake)));
       

    rightBumper.toggleOnTrue(new ParallelCommandGroup(new StartEndCommand(Intake::IntakeForward, Intake::IntakePIDOff, Intake).until(()-> Intake.GetSwitchHit()),
        new StartEndCommand(Arm::ArmIntake, Arm::ArmSubwoofer, Arm).until(()-> Intake.GetSwitchHit())));

    //   rightBumper.onFalse(new StartEndCommand(Intake::IntakeOff, Intake::IntakeOff, Intake));

    //ABXY ---------------------------------------------------------

    aButton.onTrue(new StartEndCommand(Arm::ArmIntake, Arm::ArmIntake, Arm));

    // aButton.onFalse(null);


    bButton.onTrue(new StartEndCommand(Arm::ArmSubwoofer, Arm::ArmSubwoofer, Arm));

    // bButton.onFalse(null);


    xButton.onTrue(new StartEndCommand(Arm::ArmMid, Arm::ArmMid, Arm));

    // xButton.onFalse(null);


    yButton.onTrue(new StartEndCommand(Arm::ArmAmp, Arm::ArmAmp, Arm));

    // yButton.onFalse(null);


    //D-PAD ---------------------------------------------

    //   dPadUpButton.onTrue(null);

    // dPadUpButton.onFalse(null);


    dPadDownButton.onTrue(new StartEndCommand(Arm::ArmSmartSet, Arm::ArmSmartSet, Arm).withTimeout(0));

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
        new StartEndCommand(Intake::IntakeReverse, Intake::IntakeStop, Intake)));

    backButton.onFalse(new ParallelCommandGroup( new StartEndCommand(Shooter::ShooterStop, Shooter::ShooterStop,Shooter), 
        new StartEndCommand(Intake::IntakeStop, Intake::IntakeStop, Intake)));
    



   


    // Configure the button bindings
    configureButtonBindings();

    // Adjust the dafult command based on which controler the driver ants to use.
    if (SmartDashboard.getBoolean("useXBoxController", true)) {
      driveSubsystem.setDefaultCommand(xBoxControllerCommand);
    } else {
      driveSubsystem.setDefaultCommand(flightstickCommand);
    } 

/*         Arm.setDefaultCommand(
            new RunCommand(
            () -> Arm.automaticAngle( driveSubsystem, teamAlliance.getSelected() ),
            Arm
            )
        );  */      


    // Setup all the neccicery SmartDashboard elements
    setupDashboard();

    // Configure the button bindings
    configureButtonBindings();



}

    /**
     * Configures the robot so that the controls match what was configured on SMartDashboard. (e.g 
     * whether the robot is being controlled with a flightstick or an xbox controller).
     */
    public void configureDriverPreferences() {

        // Get the drive command for the selected controller.(Note that the
        // getSelected() method returns the command associated with each option, which
        // is set above).
        driveSubsystem.setDefaultCommand(controllerSelection.getSelected());
/*         Arm.setDefaultCommand(
            new RunCommand(
            () -> Arm.automaticAngle( driveSubsystem, teamAlliance.getSelected() ),
            Arm
            )
        );  */  



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

        // Reset the IMU when the start button is pressed.
        startButton.onTrue(new ResetImuWithVisionCommand(driveSubsystem, robotVision));
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

        // Team Alliance Options
        teamAlliance.addOption("Red Alliance", Alliance.RED);
        teamAlliance.addOption("Blue Alliance", Alliance.BLUE);

        // Determines whether or not we want to run autonomous.
        SmartDashboard.putBoolean("Run Auto", false);
        SmartDashboard.putBoolean("Complex Auto", false);

        // Setup autonomous selection.
        // Loop through all of the available auto options and add each of them as a separate autonomous option
        // in SmartDashboard.
        autonomousSelection.setDefaultOption("RED_1", AutonomousOption.RED_1);
        for (AutonomousOption autonomousOption : AutonomousOption.values()) {
            autonomousSelection.addOption(autonomousOption.toString(), autonomousOption);
        }

        // Setup the ability to use complex autonomous paths

        // Add all of the configured SmartDashboard elements to the GUI.
        SmartDashboard.putData("Controller Selection", controllerSelection);
        SmartDashboard.putData("Autonomous Selection", autonomousSelection);
        SmartDashboard.putData("Alliance Selection", teamAlliance);
    }

    /**
     * rests the default commands for the robot to the selected smart dashboard values
     */
    public void updateCommands() {
        driveSubsystem.setDefaultCommand(controllerSelection.getSelected());

/*         Arm.setDefaultCommand(
            new RunCommand(
            () -> Arm.automaticAngle( driveSubsystem, teamAlliance.getSelected() ),
            Arm
            )
        );  */       
    }

    /**
     * Takes a photo using all of the cameras.
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
        
        // Setup the drivetrain to work with the selected auto.
        driveSubsystem.setAutonomous(selectedAuto);

        /*
         * Check whether or not the driver wants to run a complex autonomous path. If they do, then get
         * the complex path for the selected auto. Otherwise, make the robot follow the basic trajectory associated
         * with the selected auto.
         */
        SequentialCommandGroup selectedAutonomousRoutine = null;
        if (SmartDashboard.getBoolean("Complex Auto", false)) {

            // Get complicated autonomous routine associated with the selected auto.
            selectedAutonomousRoutine = getComplexPath(selectedAuto);
        } else {

            // Follow the basic pre-planned path
            // selectedAutonomousRoutine = new SequentialCommandGroup(
            //     new FollowTrajectoryCommand(
            //         driveSubsystem, 
            //         selectedAuto.getTrajectory())
            // );
            selectedAutonomousRoutine = new SequentialCommandGroup(
                new StartEndCommand(Arm::ArmSubwoofer,Arm::ArmSubwoofer,Arm).withTimeout(1),
                new ParallelCommandGroup(
                    new StartEndCommand(Shooter::ShooterForward, Shooter::ShooterForward,Shooter), 
                    new StartEndCommand(Intake::IntakeStop, Intake::IntakeMax, Intake).withTimeout(0.75)).withTimeout(2)
                );
        }
        // Configure the robot's settings so that it will be optimized for the selected command.
        driveSubsystem.setAutonomous(selectedAuto);

        // If we want to run autonomous, then follow the trajectory. Otherwise don't run the auto.
        if (SmartDashboard.getBoolean("Run Auto", true)) {

             // Run path following command, then stop at the end.
            return selectedAutonomousRoutine;
        }
        return null;
    }

    

    /**
     * Returns the complex autonomous routine associated with the specified autonomous routine.
     * 
     * @param autonomousOption The autonomous routine that was selected by the driver.
     * @return A {@code SequentialCommandGroup} containing a more advanced autonomous routine. A few examples of complicated
     *         autonomous routines would be autos that score multiple notes or preform more actions than driving along a
     *         single preplanned path.
     */
    private SequentialCommandGroup getComplexPath(AutonomousOption autonomousOption) {

        // Create a empty variable to store the autonomous command.
        SequentialCommandGroup complexPath;

        StartEndCommand grabNote = new StartEndCommand(Intake::IntakeMax, Intake::IntakeStop, Intake);

        ParallelCommandGroup shootNote =                         
                        new ParallelCommandGroup( 
                            new StartEndCommand(Shooter::ShooterForward, Shooter::ShooterStop, Shooter).withTimeout(3), 

                            new SequentialCommandGroup(
                                
                                new StartEndCommand(Intake::IntakeStop, Intake::IntakeStop, Intake).withTimeout(0.75),
                                new StartEndCommand(Intake::IntakeMax, Intake::IntakeStop, Intake).withTimeout(2.25)
                                
                            )

                        );
    
       
        StartEndCommand goToSubwooferPosition = new StartEndCommand(Arm::ArmSubwoofer, Arm::ArmSubwoofer, Arm);
        StartEndCommand goToIntakePosition = new StartEndCommand(Arm::ArmIntake, Arm::ArmIntake, Arm);

        SequentialCommandGroup scoreNote = new SequentialCommandGroup(

            goToSubwooferPosition.withTimeout(1.5),
            shootNote,
            goToIntakePosition.withTimeout(1.5)

        );
        
        
        // Create a complex autonomous command for each auto. See the 
        switch (autonomousOption) {
            case BLUE_1:
                complexPath = new SequentialCommandGroup(
                        //Shoot Note
                        scoreNote,
                        
                        new FollowTrajectoryCommand(driveSubsystem, // Drive up to the amp and then grab the note closest to the wall near the amp.
                            AutoConstants.BLUE_1_COMPLEX_POSITIONS.subList(0, 2)),
                        // Grab note
                        grabNote.until(() -> Intake.GetSwitchHit()),


                        new FollowTrajectoryCommand(driveSubsystem, List.of(
                            AutoConstants.BLUE_1_COMPLEX_POSITIONS.get(0), // Drive back up near the amp
                            AutoConstants.BLUE_1_STARTING_POSE)), // Drive to the original starting location

                        
                        // Score note
                        scoreNote,
                        

                        new FollowTrajectoryCommand(driveSubsystem, List.of(
                            AutoConstants.BLUE_1_COMPLEX_POSITIONS.get(0), // Drive back up near the amp
                            AutoConstants.BLUE_1_COMPLEX_POSITIONS.get(2))), // Drive to the lower of the two targeted notes
                        
                        // Grab note
                        grabNote.until(() -> Intake.GetSwitchHit() ),

                        new FollowTrajectoryCommand(driveSubsystem, List.of(
                            AutoConstants.BLUE_1_COMPLEX_POSITIONS.get(0), // Drive back up near the amp
                            AutoConstants.BLUE_1_STARTING_POSE)), // Drive to the original starting location
                        

                        // Score note
                        scoreNote
                        
                    );
                break;
            case BLUE_2:
                complexPath = new SequentialCommandGroup(
                    new FollowTrajectoryCommand(driveSubsystem, 
                        List.of(AutoConstants.BLUE_2_COMPLEX_POSITIONS.get(0))), // Drive to top note
                    
                    // Grab note
                    grabNote.until(() -> Intake.GetSwitchHit()),

                    new FollowTrajectoryCommand(driveSubsystem, 
                        List.of(FieldConstants.BLUE_ALLIANCE_SPEAKER_CENTER_SCORING_LOCATION)), // Return to the speaker
                    
                    // Shoot note
                    scoreNote,

                    new FollowTrajectoryCommand(driveSubsystem, 
                        List.of(AutoConstants.BLUE_2_COMPLEX_POSITIONS.get(1))), // Drive to center note
                    
                    // Grab note
                    grabNote.until(() -> Intake.GetSwitchHit()),

                    new FollowTrajectoryCommand(driveSubsystem, 
                        List.of(FieldConstants.BLUE_ALLIANCE_SPEAKER_CENTER_SCORING_LOCATION)), // Return to the speaker
                    
                    // Shoot note
                    shootNote,

                    new FollowTrajectoryCommand(driveSubsystem, 
                        List.of(AutoConstants.BLUE_2_COMPLEX_POSITIONS.get(2))), // Drive to bottom note
                    
                    // Grab note
                    grabNote.until(() -> Intake.GetSwitchHit()),

                    new FollowTrajectoryCommand(driveSubsystem, 
                        List.of(FieldConstants.BLUE_ALLIANCE_SPEAKER_CENTER_SCORING_LOCATION)), // Return to the speaker
                    
                    // Shoot note
                    scoreNote
                );
                break;

            case BLUE_3:
                complexPath = new SequentialCommandGroup(
                    // Score note
                    scoreNote,

                    new FollowTrajectoryCommand(driveSubsystem, 
                        AutoConstants.BLUE_3_COMPLEX_POSITIONS.subList(0, 2)),
                    // Grab note
                    grabNote.until(() -> Intake.GetSwitchHit()),

                    new FollowTrajectoryCommand(driveSubsystem, List.of(
                        AutoConstants.BLUE_3_COMPLEX_POSITIONS.get(0),
                        AutoConstants.BLUE_3_STARTING_POSE
                    )),

                    // Score note
                    scoreNote,

                    new FollowTrajectoryCommand(driveSubsystem, List.of(
                        AutoConstants.BLUE_3_COMPLEX_POSITIONS.get(0),
                        AutoConstants.BLUE_3_COMPLEX_POSITIONS.get(2)
                    )),

                    // Grab note
                    grabNote.until(() -> Intake.GetSwitchHit()),

                    new FollowTrajectoryCommand(driveSubsystem, List.of(
                        AutoConstants.BLUE_3_COMPLEX_POSITIONS.get(0),
                        AutoConstants.BLUE_3_STARTING_POSE
                    )),

                    // Score note
                    scoreNote
                );
                break;

            case RED_1:
            complexPath = new SequentialCommandGroup(
                // Shoot note
                scoreNote,

                new FollowTrajectoryCommand(driveSubsystem, // Drive up to the amp and then grab the note closest to the wall near the amp.
                    AutoConstants.RED_1_COMPLEX_POSITIONS.subList(0, 2)),
                // Grab note
                grabNote.until(() -> Intake.GetSwitchHit()),

                new FollowTrajectoryCommand(driveSubsystem, List.of(
                    AutoConstants.RED_1_COMPLEX_POSITIONS.get(0), // Drive back up near the amp
                    AutoConstants.RED_1_STARTING_POSE)), // Drive to the original starting location
                
                // Score note
                scoreNote,

                new FollowTrajectoryCommand(driveSubsystem, List.of(
                    AutoConstants.RED_1_COMPLEX_POSITIONS.get(0), // Drive back up near the amp
                    AutoConstants.RED_1_COMPLEX_POSITIONS.get(2))), // Drive to the lower of the two targeted notes
                
                // Grab note
                grabNote.until(() -> Intake.GetSwitchHit()),

                new FollowTrajectoryCommand(driveSubsystem, List.of(
                    AutoConstants.RED_1_COMPLEX_POSITIONS.get(0), // Drive back up near the amp
                    AutoConstants.RED_1_STARTING_POSE)), // Drive to the original starting location
                
                // Score note
                scoreNote
            );
            case RED_2:
                complexPath = new SequentialCommandGroup(
                    new FollowTrajectoryCommand(driveSubsystem, 
                        List.of(AutoConstants.RED_1_COMPLEX_POSITIONS.get(0))), // Drive to top note
                    // Grab note
                    grabNote.until(() -> Intake.GetSwitchHit()),

                    new FollowTrajectoryCommand(driveSubsystem, 
                        List.of(FieldConstants.RED_ALLIANCE_SPEAKER_CENTER_SCORING_LOCATION)), // Return to the speaker
                    
                    // Shoot note
                    scoreNote,

                    new FollowTrajectoryCommand(driveSubsystem, 
                        List.of(AutoConstants.RED_1_COMPLEX_POSITIONS.get(1))), // Drive to center note
                    
                    // Grab note
                    grabNote.until(() -> Intake.GetSwitchHit()),

                    new FollowTrajectoryCommand(driveSubsystem, 
                        List.of(FieldConstants.RED_ALLIANCE_SPEAKER_CENTER_SCORING_LOCATION)), // Return to the speaker
                    
                    // Shoot note
                    scoreNote,

                    new FollowTrajectoryCommand(driveSubsystem, 
                        List.of(AutoConstants.RED_1_COMPLEX_POSITIONS.get(2))), // Drive to bottom note
                    
                    // Grab note
                    grabNote.until(() -> Intake.GetSwitchHit()),

                    new FollowTrajectoryCommand(driveSubsystem, 
                        List.of(FieldConstants.RED_ALLIANCE_SPEAKER_CENTER_SCORING_LOCATION)), // Return to the speaker

                    // Shoot note
                    scoreNote

                );
                break;
            case RED_3:
                complexPath = new SequentialCommandGroup(
                    // Score note
                    scoreNote,

                    new FollowTrajectoryCommand(driveSubsystem, 
                        AutoConstants.RED_1_COMPLEX_POSITIONS.subList(0, 2)),
                    // Grab note
                    grabNote.until(() -> Intake.GetSwitchHit()),

                    new FollowTrajectoryCommand(driveSubsystem, List.of(
                        AutoConstants.RED_1_COMPLEX_POSITIONS.get(0),
                        AutoConstants.RED_1_STARTING_POSE
                    )),

                    // Score note
                    scoreNote,

                    new FollowTrajectoryCommand(driveSubsystem, List.of(
                        AutoConstants.RED_1_COMPLEX_POSITIONS.get(0),
                        AutoConstants.RED_1_COMPLEX_POSITIONS.get(2)
                    )),

                    // Grab note
                    grabNote.until(() -> Intake.GetSwitchHit()),

                    new FollowTrajectoryCommand(driveSubsystem, List.of(
                        AutoConstants.RED_1_COMPLEX_POSITIONS.get(0),
                        AutoConstants.RED_1_STARTING_POSE
                    )),

                    // Score note
                    scoreNote
                );
                break;
            default:
                complexPath = new SequentialCommandGroup(
                    new RunCommand(
                        () -> auto(), 
                        driveSubsystem)
                );
                break;
        }

        // Return the complex autonomous routine
        return complexPath;
    }

    /**
     * Start making the robot follow the selected auto path.
     */
    private void auto() {
        driveSubsystem.followTrajectory();
    }
}