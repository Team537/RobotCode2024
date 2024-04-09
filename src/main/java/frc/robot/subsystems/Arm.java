// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.ArmConstants;
import frc.utils.TalonUtils;
import frc.robot.subsystems.DriveSubsystem;
import frc.utils.Autonomous.Alliance;

public class Arm extends SubsystemBase {

  //current follower arm motor. ID 11
  public static TalonFX m_follower = new TalonFX(ArmConstants.ARM1);

  //current leader arm motor. ID 12
  public static TalonFX m_leader = new TalonFX(ArmConstants.ARM2);
  
  double setpos = 5;

  private Pose2d subwooferPose;
  private Pose2d translationAway;
  private double distanceAway = (49)/(39.3701); //Sets initial distance away to subwoofer distance. The divided by (39.2701) converts in to m

  /*
    where the arm motors are on the robot (diagram)



    intake sits on this side
    when in down position
    ___________________
    |                 |
  _______________________
  |                     |
  |                     |
  |                     |
  |---------------------| <- arm motor shaft with small sprockets
  | motor          motor| 
  |  12             11  |
  | here           here |
  |---------------------| <- arm shaft with large sprockets
  _______________________

  
  */


  //Rev Throughbore encoder, with dio port
  public static DutyCycleEncoder m_encoder = new DutyCycleEncoder(ArmConstants.ENCODER_DIO);

  // follower mode control. will auto change id. both motors go in same direction 
  // ASSUMING that the 90 degree gear boxes go the same direction (CHECK AFTER THEY ARE CHANGED)
  final Follower m_followControl = new Follower(m_leader.getDeviceID(),false);
  
  // just inits these variables, motionmagictarger relys on the init value so it doesnt move the arm to pos zero on teleop init
  public static double motionMagicTarget = m_leader.getPosition().getValue();
  public double encoderTarget = m_encoder.getAbsolutePosition();


  /** Creates a new Arm. 
  runs on robot init
  */
  public Arm() {
    //resets these variables on robot init
    motionMagicTarget = m_leader.getPosition().getValue();
    encoderTarget = m_encoder.getAbsolutePosition();

    //creates the encoder calculated target smartdash block on robot init
    SmartDashboard.putNumber("Encoder Calculated Target", 0);
    SmartDashboard.putNumber("Mid Position: ", 23);
    SmartDashboard.putNumber("Subwoofer Position: ", 10);

  }

  //Gets the desired position for motion magic, and sets both motors to the correct positions/values. 
  //runs the talonutils motionmagic command
  //the PID modifiers are in the arm constants file
  private void SetMotorsMotionMagic(double pos) {
    m_follower.setControl(m_followControl);
    TalonUtils.TalonArmMotionMagicControl(m_leader, pos);
    
    //this is for smartdashboard display
    motionMagicTarget = pos;
  }

  //Gets the desired position for PID, and sets both motors to the correct positions/values. 
  //runs the talonutils PID command
  //the PID modifiers are in the talon default constants file

  private void SetMotorsPID(double pos) {
    m_follower.setControl(m_followControl);
    TalonUtils.TalonPIDControl(m_leader, pos);
  }

  //this is for manual control, sets both motors to a percent of the max possible speed (in rps)
  private void SetMotorsVelocity(double percent) {
    TalonUtils.TalonVelocityControl(m_follower, percent);
    TalonUtils.TalonVelocityControl(m_leader, percent);
  }

  //absolute encoder calculator
  //this calculates the position that the motor needs to go to achieve the desired absolute encoder position
  private void EncoderChase(double targetENC) {
    //display on smart dashboard
    encoderTarget = targetENC;

    //current encoder position
    double currentENC = m_encoder.getAbsolutePosition();

    //calculates the new target pos
    double targetMotor = ((targetENC-currentENC)*ArmConstants.GEAR_RATIO)+(m_leader.getPosition().getValue());

    //puts to smart dashboard
    SmartDashboard.putNumber("Encoder Calculated Target", targetMotor);

    //sends the pos to the motors
    SetMotorsMotionMagic(targetMotor);
  }
  

  //Arm Motion Magic Relative Encoder

  //this segment sets the motors to a position with motion magic. 
  //remember on the robot that the total gear ratio is 200:1, but the encoder skips the 2:1 of the chain/sprocket
  //negative moves the arm up
  //when using these positions, TURN ROBOT ON WITH ARM DOWN
  //uses relative encoder
  public void ArmSubwoofer() { 
    SetMotorsMotionMagic(8);
  }

  public void ArmIntake() {
    SetMotorsMotionMagic(0);
  }
  
  public void ArmAmp() {
    SetMotorsMotionMagic(55);
  }

  public void ArmMid() {
    SetMotorsMotionMagic(20);
  }

  public void ArmSmartSet() {
    SetMotorsMotionMagic(setpos);
  }

  /**
   * Default command to set the angle of the shooter based on distance from subwoofer april tags
   * 
   * @param robotVision robotVision object used to calculate distance from tags
   * @param alliance 'b' for blue or 'r' for red. Used to decide which tag to detect
   * 
   * @author Ohihoin Vahe
   */
  
  // public void automaticAngle(DriveSubsystem driveSubsystem, Alliance teamAlliance){

  //   if (teamAlliance == Alliance.RED){
  //     subwooferPose = Constants.ArmConstants.redSubwooferPosition;
  //   }
  //   else if (teamAlliance == Alliance.BLUE){
  //     subwooferPose = Constants.ArmConstants.blueSubwooferPosition;
  //   }

  //   if (driveSubsystem.getPose() != null){

  //     translationAway = driveSubsystem.getPose().relativeTo(subwooferPose);
  //     distanceAway = Math.sqrt( Math.pow(translationAway.getX(), 2) + Math.pow(translationAway.getY(), 2) );

  //   }

  //   SetMotorsMotionMagic(angleValue(distanceAway)); //Sets the angle of the shooter using the angle value function

  // }

  /**
   * Calculates to angle value input for smart motion based on the distance from the subwoofer tag
   * 
   * @param distance how far away you are from the subwoofer tag in meters
   * @author Ohihoin Vahe
   *  
  */
  public double angleValue(double distance){

    distance *= 39.3701; // Converts meters to inches

    // angle Val uses formula created by trial and error and a regression.
    double angleVal = -38.1 + 1.71*(distance) + -0.0178*Math.pow(distance, 2) + (6.19 *Math.pow(10, -5) * Math.pow(distance, 3));

    return angleVal;

  }


  //Arm Manual
  //manually moves the arm just incase the setpositions fail
  public void ArmManualDown() {
    SetMotorsVelocity(0.2);
  }

  public void ArmManualUp() {
    SetMotorsVelocity(-0.3);
  }

  //the pid will hold the arm to position so it doesnt slowly fall
  public void ArmManualStop() {
    SetMotorsPID(m_leader.getPosition().getValue());
  } 


  //Arm Motion Magic Absolute Encoder

  //sets the motors to a desired absolute encoder position
  //uses motion magic to get to the position

  //commands for each desired position havent been made yet, but basically following
  //the format below should work, copy pasted with value changes.
  public void ChaseSet05() {
    EncoderChase(0.5);
  }

  public void ChaseSet0() {
    EncoderChase(0);
  }

  /**
   *periodic (runs every 20 ms)
   * 
   * 
  */
  @Override
  public void periodic() {
    SmartDashboard.putNumber("ENCODER POS: ", m_encoder.getAbsolutePosition());
    SmartDashboard.putNumber("Encoder Target", encoderTarget);
    SmartDashboard.putNumber("Motion Magic Arm Target", motionMagicTarget);
    SmartDashboard.putNumber("Follower Arm Positon", m_follower.getPosition().getValue());
    SmartDashboard.putNumber("Leader Arm Positon", m_leader.getPosition().getValue());

    setpos = SmartDashboard.getNumber("Set Position", 5);

    // This method will be called once per scheduler run
  }
}
