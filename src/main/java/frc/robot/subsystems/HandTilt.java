package frc.robot.subsystems;

import com.revrobotics.spark.SparkLowLevel.MotorType;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants.HandTiltConstants.Tilt;
import frc.robot.Constants.HandTiltConstants.Grab;
import frc.robot.Constants.CANSignals;
import frc.robot.Constants.ArmConst.kposition;

public class HandTilt extends SubsystemBase {
  /* THIS SECTION CREATES ALL THE EMPTY OBJECTS FOR THIS SUBSYTEM */
  // Create Motor Objects
  private final SparkMax m_handtilt;
  private final SparkMax m_grableft;
  private final SparkMax m_grabright;
  // Create Encoder Objects
  // private final RelativeEncoder m_tiltencoder;
  private final RelativeEncoder m_tiltencoder;
  // Create Closed Loop Controller Objects
  private final SparkClosedLoopController m_tiltcontrol;
  // Create Motor Configuration Objects
  private final SparkMaxConfig m_configMotor;
  private final SparkMaxConfig m_configgrableft;
  private final SparkMaxConfig m_configgrabright;
  // Create Limit Switch Objects
  private final DigitalInput m_tiltlimit;

  /* CREATE A NEW HandTilt SUBSYSTEM */
  public HandTilt() {
    /* THIS SECTION ASSIGNS STUFF TO THE CREATED OBJECTS */
    // Define the motors (Rev HD Hex Brushed plugged into Spark MAX)
    m_handtilt = new SparkMax(Tilt.kIDHandTiltMotor, MotorType.kBrushed);
    m_grableft = new SparkMax(Grab.kIDGrabbyThingy, MotorType.kBrushed);
    m_grabright = new SparkMax(Grab.kIDGrabbyThingy2, MotorType.kBrushed);

    // Define the encoders (Rev Throughbore quadrature encoders plugged into the Alt Encoder Data Port of Spark Max)
    m_tiltencoder = m_handtilt.getAlternateEncoder();

    // Define the motors configuration
    m_configMotor = new SparkMaxConfig();
    m_configMotor
        .inverted(false)
        .idleMode(IdleMode.kBrake);
    m_configMotor.alternateEncoder.apply(Tilt.kTiltEncoderConfig);
    m_configMotor.closedLoop.apply(Tilt.kTiltLoopConfig);
    m_configMotor.closedLoop.maxMotion.apply(Tilt.kMotorSmartMotion_Tilt);
    m_configMotor.softLimit.apply(Tilt.kTiltSoftLimitConfig);
    m_configMotor.signals.apply(CANSignals.HandMotors.kMotorSignalConfig_Tilt);
    
    m_configgrableft = new SparkMaxConfig();
    m_configgrableft
        .inverted(false)
        .idleMode(IdleMode.kBrake);
    m_configgrableft.signals.apply(CANSignals.HandMotors.kMotorSignalConfig_Dumb);
    //m_configgrableft.softLimit.apply(); // Set current limiting

    m_configgrabright = new SparkMaxConfig();
    m_configgrabright
        .idleMode(IdleMode.kBrake)
        .follow(m_grableft, true); // Follow the settings of the other motor.

    // Apply the motor configurations to the motors
    m_handtilt.configure(m_configMotor, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
    m_grableft.configure(m_configgrableft, ResetMode.kNoResetSafeParameters,  PersistMode.kPersistParameters);
    m_grabright.configure(m_configgrabright, ResetMode.kNoResetSafeParameters,  PersistMode.kPersistParameters);

    // Reset Encoder to Zero
    m_tiltencoder.setPosition(0.1);

    // Get the closed loop controllers from the motors
    m_tiltcontrol = m_handtilt.getClosedLoopController();

    // Configure the LOWER LIMIT limit switch.
    m_tiltlimit = new DigitalInput(Tilt.kDIOtiltdownswitch);
  }

  /**
   * GRABBER
   */
  public void open() {
    m_grableft.set(Grab.kSpeedUp);
  }
  
  /**
   * GRABBER
   */
  public void close() {
    m_grableft.set(-Grab.kSpeedDown);
  }

  /**
   * GRABBER
   */
  public void halt() {
    m_grableft.set(0.0);
  }



  /**
   * HAND TILT
   */
  public void down() {
      m_handtilt.set(0.5);
  }

  /**
   * HAND TILT
   */
  public void up() {
    m_handtilt.set(-0.5);
  }

  /**
   * HAND TILT
   */
  public void stop() {
    m_handtilt.set(0.0);
  }





  /**
   * HAND TILT
   * @return
   */
  public double getangle(){
    return m_tiltencoder.getPosition();
  }

  /**
   * HAND TILT
   * @return
   */
  public boolean getswitch(){
    return !m_tiltlimit.get(); // Lower tilt limit
  }

  /**
   * HAND TILT
   */
  public void resetAngle() {
    m_tiltencoder.setPosition(kposition.setpoint[0][3]);
  }

  /**
   * HAND TILT
   */
  public void setSmartPosition(int posID)
  {
    SmartDashboard.putNumber("Hand Setpoint", kposition.setpoint[posID][3]); //kposition.setpoint[posID][3]
    m_tiltcontrol.setReference(kposition.setpoint[posID][3], ControlType.kMAXMotionPositionControl); //kposition.setpoint[posID][3]
  }


  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    boolean switchState = getswitch();
    double motorAngle = getangle();
    
    SmartDashboard.putNumber("Tilt Angle", motorAngle);
    SmartDashboard.putBoolean("Tilt Dow Limit", switchState);
    SmartDashboard.putNumber("Tilt Current", m_handtilt.getOutputCurrent());

    // If the switch is hit and the angle isnt too far off, reset the encoder to zero.
    //
    if (switchState && ((motorAngle <= 1) && (motorAngle >= -1))) {resetAngle();}
  }
}
