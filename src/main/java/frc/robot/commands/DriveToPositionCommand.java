package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DriveSubsystem;

public class DriveToPositionCommand extends Command {
    
    private DriveSubsystem driveSubsystem;
    private Pose2d targetPosition;

    public DriveToPositionCommand(DriveSubsystem driveSubsystem, Pose2d targetPosition) {
        this.driveSubsystem = driveSubsystem;
        this.targetPosition = targetPosition;

        addRequirements(driveSubsystem);
    }

    public void execute() {
        driveSubsystem.driveToPosition(targetPosition);
    }

    
    /**
     * Stops all of the swerve modules located on the drivetrain. (All motor power is set to 0).
     */
    @Override
    public void end(boolean interrupted) {
        driveSubsystem.setX();
    }

    public boolean isFinished() {
        return false;
    }
} 
