package frc.robot.subsystems.Swerve;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveConstants;

public class SwerveDrivetrain extends SubsystemBase {
    
    private final SwerveModule FRONT_RIGHT_MODULE = new SwerveModule(
        DriveConstants.FRONT_RIGHT_DRIVE_MOTOR_PORT, 
        DriveConstants.FRONT_RIGHT_TURNING_MOTOR_PORT, 
        DriveConstants.FRONT_RIGHT_DRIVE_MOTOR_REVERSED, 
        DriveConstants.FRONT_RIGHT_TURNING_MOTOR_REVERSED, 
        1);

    private final SwerveModule FRONT_LEFT_MODULE = new SwerveModule(
        DriveConstants.FRONT_LEFT_DRIVE_MOTOR_PORT, 
        DriveConstants.FRONT_LEFT_TURNING_MOTOR_PORT, 
        DriveConstants.FRONT_LEFT_DRIVE_MOTOR_REVERSED, 
        DriveConstants.FRONT_LEFT_TURNING_MOTOR_REVERSED, 
        1);
    
    private final SwerveModule BACK_RIGHT_MODULE = new SwerveModule(
        DriveConstants.BACK_RIGHT_DRIVE_MOTOR_PORT, 
        DriveConstants.BACK_RIGHT_TURNING_MOTOR_PORT, 
        DriveConstants.BACK_RIGHT_DRIVE_MOTOR_REVERSED, 
        DriveConstants.BACK_RIGHT_TURNING_MOTOR_REVERSED, 
        1);
    
    private final SwerveModule BACK_LEFT_MODULE = new SwerveModule(
        DriveConstants.BACK_LEFT_DRIVE_MOTOR_PORT, 
        DriveConstants.BACK_LEFT_TURNING_MOTOR_PORT, 
        DriveConstants.BACK_LEFT_DRIVE_MOTOR_REVERSED, 
        DriveConstants.BACK_LEFT_TURNING_MOTOR_REVERSED, 
        1);


    public void resetHeading() {

    }
}
