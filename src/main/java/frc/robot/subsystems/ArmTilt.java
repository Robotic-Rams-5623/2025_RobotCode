// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ArmLengthConst;
import frc.robot.Constants.ArmTiltConstants;
import frc.robot.Constants.CANSignals;
import frc.robot.Constants.ArmLengthConst.kPositions;

public class ArmTilt extends SubsystemBase {
  /** Creates a new ArmTilt. */
  private final SparkMax m_armtilt;
  private final SparkMaxConfig m_configMotor;
  private final DigitalInput m_baseExtendLimit;
  private final SparkClosedLoopController m_basecontrol;
  private final RelativeEncoder m_baseencoder;

  private TrapezoidProfile m_Profilebot;
  private TrapezoidProfile.State goalbot;
  private TrapezoidProfile.State setpointbot;
  private Timer m_Timer;
  private double m_setpointbot;


  public ArmTilt() {
    m_armtilt = new SparkMax(ArmTiltConstants.kIDArmTiltMotor, MotorType.kBrushed);

    m_baseencoder = m_armtilt.getAlternateEncoder();
    m_baseencoder.setPosition(0.0);

    m_configMotor = new SparkMaxConfig();
    m_configMotor
        .inverted(false)
        .idleMode(IdleMode.kBrake)
        .openLoopRampRate(0.1)
        .closedLoopRampRate(0.1);
    m_configMotor.encoder.apply(ArmLengthConst.kMotorEncoderConfig);
    m_configMotor.closedLoop.apply(ArmLengthConst.kMotorLoopConfig);
    m_configMotor.softLimit.apply(ArmLengthConst.kMotorSoftLimitConfig);
    m_configMotor.signals.apply(CANSignals.ArmMotors.kMotorSignalConfig);

    m_armtilt.configure(m_configMotor, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);

    m_basecontrol = m_armtilt.getClosedLoopController();
    
    m_baseExtendLimit = new DigitalInput(ArmLengthConst.kDIOBaseExtendSwitch);

    m_Timer = new Timer();
    m_Timer.start();
    m_Timer.reset();
    
    m_setpointbot = kPositions.setpoint[0][1];

    m_Profilebot = new TrapezoidProfile(ArmLengthConst.kArmMotionConstraint);
    
    goalbot = new TrapezoidProfile.State();
    setpointbot = new TrapezoidProfile.State();

    updateMotionprofile();
  }

  public void setTargetPosition(double setBot) {
    if ( setBot != m_setpointbot) {
      m_setpointbot = setBot;
      updateMotionprofile();
    }
  }

  private void updateMotionprofile(){
    TrapezoidProfile.State statebot = new TrapezoidProfile.State(m_baseencoder.getPosition(), m_baseencoder.getVelocity());
    TrapezoidProfile.State goalbot = new TrapezoidProfile.State(m_setpointbot, 0.0);
    
    m_Timer.reset();
  }

  public void runAutomatic() {
    double elapsedTime = m_Timer.get();
    if (m_Profilebot.isFinished(elapsedTime)) {
      setpointbot = new TrapezoidProfile.State(m_setpointbot, 0.0);
    }
    else {
      setpointbot = m_Profilebot.calculate(elapsedTime, setpointbot, goalbot);
    }

    // feedforward = Constants.Arm.kArmFeedforward.calculate(m_encoder.getPosition()+Constants.Arm.kArmZeroCosineOffset, targetState.velocity);
    m_basecontrol.setReference(setpointbot.position, ControlType.kPosition);
  }

  public void up() {
    m_armtilt.set(ArmTiltConstants.kSpeedUp);
  }

  public void down() {
    m_armtilt.set(-ArmTiltConstants.kspeedDown);
  }

  public void halt() {
    m_armtilt.set(0.0); 
  }

  public boolean getbottomswitch(){
    return m_baseExtendLimit.get();
  }

  public void resetBaseEncoder() {
    m_baseencoder.setPosition(0);
  }

  public double getPosition(){
    return m_baseencoder.getPosition();
  }

  public void setArmPosition(int posID)
  {
    /**
     * Set the position of both arms simultaneously to move in fluid motion to desired position.
     * A feedforward could be calculated and added to the top control if gravity starts to fight us.
     */
     //m_basecontrol.setReference(kPositions.setpoint[posID][0], ControlType.kPosition);
    m_basecontrol.setReference(kPositions.setpoint[posID][1], ControlType.kPosition);
  }



  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("get arm length bot ", getPosition());
    SmartDashboard.putBoolean("arm bottom switch", getbottomswitch());
  }
}
