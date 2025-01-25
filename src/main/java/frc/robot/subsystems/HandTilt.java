// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkAbsoluteEncoder;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.HandTiltConstants;

public class HandTilt extends SubsystemBase {
  /** Creates a new HandTilt. */
  private final SparkMax m_handtilt;
  private final SparkMaxConfig m_configMotor;
  private final SparkAbsoluteEncoder m_tiltencoder;
  private final SparkClosedLoopController m_tiltcontrol;


  public HandTilt() {
    m_handtilt = new SparkMax(HandTiltConstants.kIDHandTiltMotor, MotorType.kBrushed);
    m_configMotor = new SparkMaxConfig();
    m_tiltencoder = m_handtilt.getAbsoluteEncoder();

    m_configMotor
        .inverted(false)
        .idleMode(IdleMode.kBrake);
    m_configMotor.absoluteEncoder
        .positionConversionFactor(0.1)
        .velocityConversionFactor(1000)
        .zeroOffset(0)
        .inverted(false);
    m_configMotor.closedLoop
        .feedbackSensor(FeedbackSensor.kAbsoluteEncoder)
        .outputRange(-0.4, 0.4)
        .pidf(0.001, 0, 0, 0.001);
    
    m_handtilt.configure(m_configMotor, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);
    m_tiltcontrol = m_handtilt.getClosedLoopController();
  }

  public void up() {
    m_handtilt.set(HandTiltConstants.kSpeedUp);
  }

  public void down() {
    m_handtilt.set(-HandTiltConstants.kSpeedDown);
  }

  public void stop() {
    m_handtilt.set(0.0);
  }

  public void setAngle(double angle) {
    m_tiltcontrol.setReference(angle, ControlType.kPosition);
  }
  
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
