// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants.ArmConst.Extend;
import frc.robot.Constants.ArmConst.MotorConfigs;
import frc.robot.Constants.CANSignals;
import frc.robot.Constants.ArmConst.kposition;


public class ArmExtend extends SubsystemBase {
  /* THIS SECTION CREATES ALL THE EMPTY OBJECTS FOR THIS SUBSYTEM */
  // Create Motor Objects
  private final SparkMax m_extend;
  // Create Encoder Objects
  // private final RelativeEncoder m_encoder;
  private final RelativeEncoder m_encoder;
  // Create Closed Loop Controller Objects
  private final SparkClosedLoopController m_control;
  // Create Motor Configuration Objects
  private final SparkMaxConfig m_configmotor;
  // Create Limit Switch Objects
  private DigitalInput m_retractlimit;
  
  /* CREATE A NEW ArmExtend SUBSYSTEM */
  public ArmExtend() {
    /* THIS SECTION ASSIGNS STUFF TO THE CREATED OBJECTS */
    // Define the motors (Rev HD Hex Brushed plugged into Spark MAX)
    m_extend = new SparkMax(Extend.kIDextend, MotorType.kBrushed);

    // Define the encoders (Built-in Rev HD Hex Motor quadrature encoder plugged into the Alt Encoder Data Port of Spark Max)
    m_encoder = m_extend.getAlternateEncoder();

    // Define the motor's configuration
    m_configmotor = new SparkMaxConfig();
    m_configmotor
        .inverted(false)
        .idleMode(IdleMode.kBrake);
    m_configmotor.alternateEncoder.apply(MotorConfigs.kAltEncoderConfig_HD);
    m_configmotor.closedLoop.apply(MotorConfigs.kMotorLoopConfig_HD);
    m_configmotor.softLimit.apply(MotorConfigs.kMotorSoftLimitConfig_Extend);
    m_configmotor.signals.apply(CANSignals.ArmMotors.kMotorSignalConfig);
    
    // Apply the motor configurations to the motors
    m_extend.configure(m_configmotor, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);
    
    // Reset Encoder to Zero
    m_encoder.setPosition(kposition.setpoint[0][2]);

    // Get the closed loop controllers from the motors
    m_control = m_extend.getClosedLoopController();

    // Configure the LOWER LIMIT limit switch.
    m_retractlimit = new DigitalInput(Extend.kDIOextendretractswitch);
  }

  /**
   * 
   */
  public void out(){
    m_extend.set(Extend.kSpeedUp);
  }

  /**
   * 
   */
  public void in(){
    if(getSwitch()) {
      stop();
    } else {
      m_extend.set(-Extend.kspeedDown);
    }
  }

  /**
   * 
   */
  public void stop(){
    m_extend.set(0.0);
  }

  /**
   * 
   */
  public void resetencoder(){
    m_encoder.setPosition(kposition.setpoint[0][2]);
  }

  /**
   * 
   * @return
   */
  public double getPosition() {
    return m_encoder.getPosition();
  }

  /**
   * 
   * @param posID
   */
  public void setArmPosition(int posID){
    m_control.setReference(kposition.setpoint[posID][2], ControlType.kPosition);
  }

  /**
   * 
   * @param posID
   */
  public void setSmartPosition(int posID)
  {
    m_control.setReference(kposition.setpoint[posID][2], ControlType.kMAXMotionPositionControl);
  }

  /**
   * 
   * @return
   */
  public boolean getSwitch(){
    return !m_retractlimit.get();
  }


  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    double position = getPosition();
    boolean proxSwitch = getSwitch();

    SmartDashboard.putNumber("arm extend position", position);
    SmartDashboard.putBoolean("arm extend switch", proxSwitch);

    if (proxSwitch) {resetencoder();}
  }
}
