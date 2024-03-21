// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.Pigeon2;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;

import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ArmConstants;

public class Arm extends SubsystemBase {

  public static TalonFX m_arm1 = new TalonFX(ArmConstants.ARM1);
  public static TalonFX m_arm2 = new TalonFX(ArmConstants.ARM2);
  public static DutyCycleEncoder m_encoder = new DutyCycleEncoder(ArmConstants.ENCODER_DIO);

  final Follower m_follower = new Follower(ArmConstants.ARM2,false);

  double driveGyroYaw = 0;  // arm pigeon code, not used

  public double ChaseTarget = m_encoder.getAbsolutePosition();

  boolean active = false;

  // just inits these variables, targetPos relys on the armgetpos so it doesnt move the arm to pos zero on teleop init
  public static double FalconArmTarget = m_arm1.getPosition().getValue();
  public static double EncoderTarget = m_arm1.getPosition().getValue();




  /** Creates a new Arm. */
  public Arm() {
  }

  /* Pigeon on arm code
  private double ConvertAngleToRot() {
    double rotations = (m_pigeon.getAngle()/360)*200;
    return rotations;
  }
    private double ConvertRotToAngle() {
    double angle = (m_arm1.getPosition().getValue()/200)*360;
    return angle;
  }*/

  private void TalonfxMotionMagicSlots(double targetPos) {
    double velocity = getTargetDir(targetPos);
    var talonFXConfigs = new TalonFXConfiguration();

    // set slot 0 gains
    var slot1Configs = talonFXConfigs.Slot1;
    slot1Configs.kV = 0.17; // A velocity target of 1 rps results in [var] V output
    slot1Configs.kA = 0.04; // An acceleration of 1 rps/s requires [var] V output
    slot1Configs.kP = 1; // A position error of 2.5 rotations results in 12 V output
    slot1Configs.kI = 1; // no output for integrated error
    slot1Configs.kD = 0.1; // A velocity error of 1 rps results in 0.1 V output
    
    //gear ratio
    talonFXConfigs.Feedback.SensorToMechanismRatio = ArmConstants.GEAR_RATIO;

    //configs slot1Configs.kg to be variable (arm cosine, with high gravity being at high)
    talonFXConfigs.withSlot1(slot1Configs.withGravityType(GravityTypeValue.Arm_Cosine));

    
    // set Motion Magic settings
    var motionMagicConfigs = talonFXConfigs.MotionMagic;
    motionMagicConfigs.MotionMagicCruiseVelocity = velocity; // Target cruise velocity of 80 rps
    motionMagicConfigs.MotionMagicAcceleration = 40; // Target acceleration of 160 rps/s (0.5 seconds)
    motionMagicConfigs.MotionMagicJerk = 100; // Target jerk of 1600 rps/s/s (0.1 seconds)

    m_arm1.getConfigurator().apply(talonFXConfigs);
    m_arm2.getConfigurator().apply(talonFXConfigs);

  }

  private double getTargetDir(double targetPos) {
    double currentPos = m_arm2.getPosition().getValue();
    double velocity = 0;
    if (targetPos < currentPos) {
      //up
      velocity = 60;
    } else if (targetPos > currentPos) {
      velocity = 40;
    } else if (targetPos == currentPos) {
      velocity = 0;
    }
    return velocity;
  }

  private String getTargetDirChase(double targetPos) {
    double currentPos = m_arm2.getPosition().getValue();
    String direction = "";
    if (targetPos < currentPos) {
      //up
      direction ="up";
    } else if (targetPos > currentPos) {
        direction = "down";
    } else if (targetPos == currentPos) {
      direction = "none";
    }
    return direction;
  }

  private MotionMagicVoltage TalonfxMotionMagic(double pos) {
    TalonfxMotionMagicSlots(pos);
    MotionMagicVoltage m_request = new MotionMagicVoltage(pos).withSlot(1);
    return m_request;
  }

  private void SetMotorsMotionMagic(double pos) {
    m_arm1.setControl(m_follower);
    m_arm2.setControl(TalonfxMotionMagic(pos));
  }

  public void ArmSubwoofer() { 
    SetMotorsMotionMagic(-7);
    FalconArmTarget = -7;
    // SetMotorsMotionMagic(findTargetDist(-7));
    active = false;
  }

  public void ArmIntake() {
    SetMotorsMotionMagic(0);
    FalconArmTarget = 0;
    // SetMotorsMotionMagic(findTargetDist(0));
    active = false;
  }
  
  public void ArmAmp() {
    SetMotorsMotionMagic(-55);
    FalconArmTarget = -55;
    // SetMotorsMotionMagic(findTargetDist(-55));
    active = false;
  }

  public void ArmMid() {
    SetMotorsMotionMagic(-23);
    FalconArmTarget = -23;
    // SetMotorsMotionMagic(findTargetDist(-23));
    active = false;
  }

  public void ArmMotionMagicStop() {
    SetMotorsMotionMagic(FalconArmTarget);
    active = false;
  }

  public void ArmManualDown() {
    m_arm1.set(0.2);
    m_arm2.set(0.2);
    active = false;
  }

  public void ArmManualUp() {
    m_arm1.set(-0.3);
    m_arm2.set(-0.3);
    active = false;
  }

  public void ArmManualStop() {
    SetMotorsMotionMagic(m_arm2.getPosition().getValue());
    active = false;
  } 

  public void ChaseSet2() {
    active = true;
    ChaseTarget = 0.5;
  }

  public void ChaseSet0() {
    active = true;
    ChaseTarget = 0;
  }
  
  public boolean targetPid() {
    active = false;
    if ((FalconArmTarget - 0.2) < m_arm2.getPosition().getValue() && m_arm2.getPosition().getValue() < FalconArmTarget + 0.2) {
      // if the motor value is within 0.2[Deadband] rot of the desired end location, enable the PID loop
      // and override the motionmagic trapezoidal loop. the PID loop is stronger at keeping the 
      // arm in position than the motion magic loop. 
      return true; 
    } else {
      return false;
    }
  }

  private void encoderChase(double targetENC) {
    /** target is encoder POS
    if target is 0.81 ENC
    Motor wants 0.81*gear ratio
    testing uses gear ratio of 3
    so we want the arm to end up at 2.43 rot
    newtarget = ArmConstants.GEAR_RATIO * target
    new motor target is now 2.43

    however, the motor/chain may have slipped and 0 on the motor may not line up with 0 on the encoder.
    we must account for this by adding the current motor pos.
    this accounts for the slip
    remember, encoder target is 0.81 (motor target is 2.43), but in this case the motor positon is 1 when encoder is 0
        3.43        2.43        1   
    newtarget2 = newtarget + m_arm2.getPosition().getValue();

    so this should work right? but what if it slips in transit? we must keep updating the pos.
    thats where the run command comes in help
    RunChase will constantly set the MotionMagic to a new position which will account for any slippage.

    lets run it with no offset
    when motor = 0(/3), ENC = 0.6
    we cant go more than one ROT on the abs encoder as it will screw it up
    target ENC = 0.25, which will make the motor = 0.75, or 0.25*3
    but this is bad. the ENC needs to go down, but the motor will go up. this is where directional comes in

    currentmotor = 0, ENC = 0.6

    targetENC = 0.25, 
    targetMotor = 0.75 = (0.25*3) = (targetENC * GearRatio)
    if (dir = "down") {
      targetMotor *= -1
      motor will now go -0.75
    }

    this solves that issue of the reversed direction
    lets run the all the calcs 

    currentmotor = 0, currentENC = 0.6

    targetENC = 0.25
    targetmotor = 0.75 = 0.25*3 = targetENC*gearratio

    if (targetENC < currentENC && targetMotor > currentMotor) {
      targetMotor *= 1;
    }

    targetMotor + m_arm2.getPosition().getValue();

    since the motor is at zero the offset thingy doesnt really do anything
    now we run it with a different starting motor pos

    currentmotor = 1, currentENC = 0.6

    targetENC = 0.25
    targetmotor = 0.75 = 0.25*3 = targetENC*gearratio

    if (targetENC < currentENC && targetMotor > currentMotor) {
      targetMotor *= 1;
      //now -0.75
    }
 
    targetMotor += currentmotor;

    -.75+1
    = 0.25
    will go 0.75 rotations in the negative direction
    end up at 0.25



    */
    if (active == true) {
      double currentENC = m_encoder.getAbsolutePosition();
      double currentOffsetENC = m_encoder.getAbsolutePosition()-ArmConstants.ENCODER_OFFSET;

      targetENC = targetENC; //parameter
      double targetMotor = targetENC*ArmConstants.GEAR_RATIO;

      if (targetENC < currentENC && targetMotor> m_arm2.getPosition().getValue() ) {//m_arm2.getpos = currentmotor
        targetMotor *= -1;
      }

      targetMotor += m_arm2.getPosition().getValue();//m_arm2.getpos = currentmotor

      //sends the pos to the motors
      SetMotorsMotionMagic(targetENC);
    }
  }

  public void RunChase() {
    encoderChase(ChaseTarget);
  }


  @Override
  public void periodic() {
    SmartDashboard.putNumber("ABSOLUTE POS", m_encoder.getAbsolutePosition());

    SmartDashboard.putNumber("CHASE TARGET", ChaseTarget);

    SmartDashboard.putNumber("ABSOLUTE POS OFF", m_encoder.getAbsolutePosition()-ArmConstants.ENCODER_OFFSET);
    SmartDashboard.putNumber("EncoderTarget", EncoderTarget);

    // SmartDashboard.putNumber("ArmPitch", m_armPigeon.getPitch().getValue());
    // SmartDashboard.putNumber("ArmYaw", m_armPigeon.getYaw().getValue());
    SmartDashboard.putBoolean("withinPosRange", targetPid());
    SmartDashboard.putNumber("Falcon Arm Target", FalconArmTarget);
    SmartDashboard.putNumber("ARM POS 1", m_arm1.getPosition().getValue());
    SmartDashboard.putNumber("ARM POS 2", m_arm2.getPosition().getValue());

    // This method will be called once per scheduler run
  }
}
