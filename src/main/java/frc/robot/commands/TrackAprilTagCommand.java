package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.AutoConstants;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.cameras.RobotVision;
import frc.utils.geometry.TagPose3d;

public class TrackAprilTagCommand extends Command {
    
    // Controllers
    private PIDController xController = new PIDController(
        AutoConstants.X_CONTROLLER_KP, 0, 0);
    private PIDController yController = new PIDController(
        AutoConstants.Y_CONTROLLER_KP, 0, 0);
    private PIDController thetaController = new PIDController(
        AutoConstants.THETA_CONTROLLER_KP, 0, 0);

    // Subsystems
    private DriveSubsystem swerveDrivetrain;
    private RobotVision vision;

    public TrackAprilTagCommand(DriveSubsystem driveSubsystem, RobotVision robotVision) {

        // Gain access to subsystems
        this.swerveDrivetrain = driveSubsystem;
        this.vision = robotVision;

        // Configure PID Controllers
        xController.setTolerance(.33); // Drive within 1/3 a meter of the target. Helps ensure the robot can reach target.
        xController.setSetpoint(0); // Try to get the distance to the tag to 0.

        yController.setTolerance(.33);
        yController.setSetpoint(0);

        thetaController.setTolerance(.1);
        thetaController.setSetpoint(0);

        // Prevent damage to the robot
        addRequirements(swerveDrivetrain, vision);
    }

    @Override
    public void initialize() {
        xController.reset();
        yController.reset();
        thetaController.reset();
    }

    @Override
    public void execute() {
        
        // Calculate distance to the desired tag. Return if we can't see the desired tag.
        TagPose3d distanceToTag = vision.getDistanceToTag(2);

        // Stop moving if we can't see a tag to ensure that the person holding the tag isn't
        // injured in poor lighting or when the tag becomes obstructed.
        if (distanceToTag == null) { 
            swerveDrivetrain.setX();
            return; 
        }

        /*
         * Multiply all of these values by negative one so that the PID controller will 
         * drive towards the tag. Were this line of code not to be here, the robot would 
         * attempt to drive in the complete opposite direction of the tag as a result of the
         * PID controllers attempting to lower the distance, which results in a negative speed
         * and thus causes the robot to drive backwards away from the tag.
         */
        distanceToTag.times(-1);

        // Ensure that the robot doesn't drive directly on top of the tag. This ensures that the 
        // robot doesn't hit the person holding the tag.
        distanceToTag.setX(distanceToTag.getX() + .5); // In meters

        // Calculate speed
        var xSpeed = xController.calculate(distanceToTag.getX());
        var ySpeed = yController.calculate(distanceToTag.getY());
        var thetaSpeed = thetaController.calculate(distanceToTag.getYaw());

        // Drive the robot towards the tag.
        swerveDrivetrain.drive(xSpeed, ySpeed, thetaSpeed, 0, false, true);
    }

    /**
     * Stops all of the swerve modules located on the drivetrain. (All motor power is set to 0).
     */
    @Override
    public void end(boolean interrupted) {
        swerveDrivetrain.setX();
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}