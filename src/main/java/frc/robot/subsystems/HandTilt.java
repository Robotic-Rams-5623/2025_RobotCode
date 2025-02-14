// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkLowLevel.MotorType;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.HandTiltConstants;

public class HandTilt extends SubsystemBase {
  /** Creates a new HandTilt. */
  private final SparkMax m_handtilt;
  private final SparkMaxConfig m_configMotor;
  private final RelativeEncoder m_tiltencoder;
  private final SparkClosedLoopController m_tiltcontrol;

  private final SparkMax m_grableft;
  private final SparkMax m_grabright;
  private final SparkMaxConfig m_configgrableft;
  private final SparkMaxConfig m_configgrabright;
  // DIGITAL INPUT FOR TOP LIMIT
  private final DigitalInput m_tiltlimit;



  public HandTilt() {
    m_handtilt = new SparkMax(HandTiltConstants.Tilt.kIDHandTiltMotor, MotorType.kBrushed);
    m_configMotor = new SparkMaxConfig();
    m_tiltencoder = m_handtilt.getEncoder();

    m_configMotor
        .inverted(false)
        .idleMode(IdleMode.kBrake);
    m_configMotor.encoder.apply(HandTiltConstants.Tilt.kTiltEncoderConfig);
    m_configMotor.closedLoop.apply(HandTiltConstants.Tilt.kTiltLoopConfig);
    m_configMotor.softLimit.apply(HandTiltConstants.Tilt.kTiltSoftLimitConfig);

    m_handtilt.configure(m_configMotor, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);
    m_tiltcontrol = m_handtilt.getClosedLoopController();

    m_grableft = new SparkMax(HandTiltConstants.Grab.kIDGrabbyThingy, MotorType.kBrushed);
    m_grabright = new SparkMax(HandTiltConstants.Grab.kIDGrabbyThingy2, MotorType.kBrushed);

    m_configgrableft = new SparkMaxConfig();
    m_configgrableft
        .inverted(false)
        .idleMode(IdleMode.kBrake);
    m_configgrabright = new SparkMaxConfig();
    m_configgrabright
        .idleMode(IdleMode.kBrake)
        .follow(m_grableft, true);

    m_grableft.configure(m_configgrableft, ResetMode.kNoResetSafeParameters,  PersistMode.kPersistParameters);
    m_grabright.configure(m_configgrabright, ResetMode.kNoResetSafeParameters,  PersistMode.kPersistParameters);
    m_tiltlimit = new DigitalInput(HandTiltConstants.Tilt.kDIOtiltdownswitch);
  }

  public void up() {
    // m_handtilt.set(HandTiltConstants.kSpeedUp);
  }

  public void down() {
    // m_handtilt.set(-HandTiltConstants.kSpeedDown);
  }

  public void stop() {
    // m_handtilt.set(0.0);
  }

  public void setAngle(double angle) {
    // m_tiltcontrol.setReference(angle, ControlType.kPosition);
  }

  public void open() {
    m_grableft.set(HandTiltConstants.Grab.kSpeedUp);
  }
  
  public void close() {
    m_grableft.set(-HandTiltConstants.Grab.kSpeedDown);
  }

  public void halt() {
    m_grableft.set(0.0);
  }

  public double getangle(){
    return m_tiltencoder.getPosition();

  }

  public boolean getswitch(){
    return m_tiltlimit.get();
  }


  @Override
  public void periodic() {
    // This method will be called once per scheduler run

    SmartDashboard.putNumber("tiltencoder", getangle());
    SmartDashboard.putBoolean("tiltlimit", getswitch());
  }
}
