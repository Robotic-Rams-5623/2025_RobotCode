// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkRelativeEncoder;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ArmLengthConstants;

public class ArmLength extends SubsystemBase {
  // /** Creates a new ArmLength. */
  // private final SparkMax m_armbase;
  // private final SparkMax m_armtop;
  // private final RelativeEncoder m_baseencoder;
  // private final SparkClosedLoopController m_basecontrol;
  // private final SparkClosedLoopController m_topcontrol;
  // private final SparkMaxConfig m_configmotor;

  public ArmLength() {
    // m_armbase = new SparkMax(ArmLengthConstants.kIDArmBaseLength, MotorType.kBrushed);
    // m_armtop = new SparkMax(ArmLengthConstants.kIDArmTopLength, MotorType.kBrushed);
    // m_configmotor = new SparkMaxConfig();
    // m_baseencoder = m_armbase.getEncoder();
    
    // m_configmotor
    //     .inverted(false)
    //     .idleMode(IdleMode.kBrake);
    //     m_configmotor.encoder
    //     .positionConversionFactor(0.1)
    //     .velocityConversionFactor(1000)
    //     .inverted(false);
    // m_configmotor.closedLoop
    //     .feedbackSensor(FeedbackSensor.kAbsoluteEncoder)
    //     .outputRange(-0.4, 0.4)
    //     .pidf(0.001, 0, 0, 0.001);

    // m_armbase.configure(m_configmotor, null, null);
    // m_armtop.configure(m_configmotor, null, null);

    // m_topcontrol = m_armtop.getClosedLoopController();
    // m_basecontrol = m_armbase.getClosedLoopController();
  } 

  public void baseUp(){
    // m_armbase.set(ArmLengthConstants.kSpeedUp);
  }

  public void baseDown(){
    // m_armbase.set(-ArmLengthConstants.kSpeedDown);
  }

  public void baseStop(){
    // m_armbase.set(0.0);
  }

  public void topUp(){
    // m_armtop.set(ArmLengthConstants.kSpeedUp);
  }

  public void topDown(){
    // m_armtop.set(-ArmLengthConstants.kSpeedDown);
  }

  public void topHalt(){
    // m_armtop.set(0.0);
  }

  public void resetencoder() {
    // m_baseencoder.setPosition(0);
  }
  
  public void setbaseheight(double height){
    // m_basecontrol.setReference(height, ControlType.kPosition);
  }

  public void settopheight(double height){
    // m_topcontrol.setReference(height, ControlType.kPosition);
  }

  public void setbasespeed(double speed){
    // m_armbase.set(speed);
  }
  
  public void settopspeed(double speed){
    // m_armtop.set(speed);
  }
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
