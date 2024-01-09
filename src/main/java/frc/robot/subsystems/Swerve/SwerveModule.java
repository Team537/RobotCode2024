package frc.robot.subsystems.Swerve;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.SwerveModuleConstants;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.SparkMaxAbsoluteEncoder.Type;

public class SwerveModule {

    private CANSparkMax driveMotor;
    private CANSparkMax turningMotor;

    private RelativeEncoder driveEncoder;
    private AbsoluteEncoder turningEncoder;

    private PIDController turningPidController;
    private double angularOffset;

    /**
     * 
     * @param driveMotorId            The port that the drive motor is connected to.
     * @param turningMotorId          The port that the turning motor is connected
     *                                to.
     * @param driveMotorReversed      Whether or not the drive is reversed.
     * @param turningMotorReversed    Whether or not the
     * @param absaluteEncoderId       The port that the absalute encoder is located
     *                                at.
     * @param absaluteEncoderOffset
     * @param absaluteEncoderReversed
     */
    public SwerveModule(int driveMotorId, int turningMotorId, boolean driveMotorReversed, boolean turningMotorReversed,
            double absaluteEncoderOffset) {

        // Create the motors for the swerve module.
        driveMotor = new CANSparkMax(driveMotorId, MotorType.kBrushless);
        turningMotor = new CANSparkMax(driveMotorId, MotorType.kBrushless);

        driveMotor.setInverted(driveMotorReversed);
        turningMotor.setInverted(turningMotorReversed);

        driveEncoder = driveMotor.getEncoder();
        turningEncoder = turningMotor.getAbsoluteEncoder(Type.kDutyCycle);
        angularOffset = absaluteEncoderOffset;
        
        driveEncoder.setPositionConversionFactor(SwerveModuleConstants.DRIVE_MOTOR_TICKS_PER_METER);
        driveEncoder.setVelocityConversionFactor(SwerveModuleConstants.DRIVE_MOTOR_TICKS_PER_METER);
        turningEncoder.setPositionConversionFactor(SwerveModuleConstants.DRIVE_MOTOR_TICKS_PER_METER);
        turningEncoder.setVelocityConversionFactor(SwerveModuleConstants.DRIVE_MOTOR_TICKS_PER_METER);

        // Create a PID controler tuned using the
        turningPidController = new PIDController(SwerveModuleConstants.kp, SwerveModuleConstants.ki,
                SwerveModuleConstants.kd);
        turningPidController.enableContinuousInput(-Math.PI, Math.PI);

        // Reset the encoders
        resetEncoders();
    }

    /**
     * 
     * @param state 
     */
    public void setDesiredState(SwerveModuleState state) {

        /*
         * Check and see if the speed we are trying to mve is incredibly insugnificant. If it is, then stop
         * all of the motors. This ensures that the turning motor doesn't reset it's rotation to 0 degrees
         * when we stop pushing the button.
         */
        if (Math.abs(state.speedMetersPerSecond) < 0.001) {
            stop();
            return;
        }

        // Optimize the angle setpoint so that we never have to rotate more than 90 degrees.
        state = SwerveModuleState.optimize(state, getState().angle);

        // Scale the velocity down using the robot's max speed and set it to the drive motor's speed.
        driveMotor.set(state.speedMetersPerSecond / DriveConstants.MAX_DIVE_SPEED_METERS_PER_SECOND);

        // Calculate the turning motor's speed using the PID controller.
        turningMotor.set(turningPidController.calculate(getTurningPosition(), state.angle.getRadians()));

        /*
         * ... Debug code would go here ...
         */
    }

    /**
     * This method sets the power of all of the motors in this swerve module to 0.
     */
    public void stop() {
        driveMotor.set(0);
        turningMotor.set(0);
    }

    /**
     * Returns a new SwerveModuleState initalized with the drive motor's velocity and 
     * turning motor's position.
     * 
     * @return A new SwerveModuleState containg the drive motor's velocity and 
     * turning motor's position.
     */
    public SwerveModuleState getState() {
        return new SwerveModuleState(getDriveVelocity(), new Rotation2d(getTurningPosition()));
    }

    /**
     * Returns the position of the drive motor in this swerve module.
     *
     * @return The position of the drive motor's encoder in meters.
     */
    public double getDrivePosition() {
        return driveEncoder.getPosition();
    }

    /**
     * Returns the angle the turning motor in this swerve module is facing.
     * 
     * @return The angle that the turning motor is facing in radians.
     */
    public double getTurningPosition() {
        return turningEncoder.getPosition();
    }

    /**
     * Returns the velocity of the drive motor in this swerve module.
     * 
     * @return The velocity of the drive motor in meters per second.
     */
    public double getDriveVelocity() {
        return driveEncoder.getVelocity();
    }

    /**
     * Returns the angular velocity of the turning motor in this swerve module.
     * 
     * @return The angular velocity of the turning motor in radians per second.
     */
    public double getTurningVelocity() {
        return turningEncoder.getVelocity();
    }

    /**
     * This method resets the encoder values for this module's motors.
     */
    public void resetEncoders() {
        driveEncoder.setPosition(0);
    }
}
