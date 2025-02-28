// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.math.MathUtil;

import frc.robot.Constants.ArmConst.Tilt;
import frc.robot.Constants.ArmConst.MotorConfigs;
import frc.robot.Constants.CANSignals;
import frc.robot.Constants.ArmConst.kposition;

public class ArmTilt extends SubsystemBase {
  /* THIS SECTION CREATES ALL THE EMPTY OBJECTS FOR THIS SUBSYTEM */
  // Create Motor Objects
  private final SparkMax m_armtilt;
  // Create Encoder Objects
  // private final RelativeEncoder m_baseencoder;
  private final RelativeEncoder m_encoder;
  // Create Closed Loop Controller Objects
  private final SparkClosedLoopController m_control;
  // Create Motor Configuration Objects
  private final SparkMaxConfig m_configMotor;
  // Create Limit Switch Objects
  private final DigitalInput m_backwardLimit;
  // private final DigitalInput m_forwardLimit;
 
  // Subsystem Variables
  private boolean proxSwitch_lastState;

  /* CREATE A NEW ArmTilt SUBSYSTEM */
  public ArmTilt() {
    /* THIS SECTION ASSIGNS STUFF TO THE CREATED OBJECTS */
    // Define the motors (NEO Brushless plugged into Spark MAX)
    m_armtilt = new SparkMax(Tilt.kIDArmTiltMotor, MotorType.kBrushless);

    // Define the encoders (Rev Throughbore quadrature encoders plugged into the Alt Encoder Data Port of Spark Max)
    m_encoder = m_armtilt.getAlternateEncoder();
    
    // Define the motors configuration
    m_configMotor = new SparkMaxConfig();
    m_configMotor
        .inverted(true)
        .idleMode(IdleMode.kBrake)
        .openLoopRampRate(0.1);
    m_configMotor.alternateEncoder.apply(MotorConfigs.kAltEncoderConfig_NEO);
    m_configMotor.closedLoop.apply(MotorConfigs.kMotorLoopConfig_Bot);
    m_configMotor.closedLoop.maxMotion.apply(MotorConfigs.kMotorSmartMotion_Bot);
    // m_configMotor.softLimit.apply(MotorConfigs.kMotorSoftLimitConfig_Base);
    m_configMotor.signals.apply(CANSignals.ArmMotors.kMotorSignalConfig);
    
    // Apply the motor configurations to the motors
    m_armtilt.configure(m_configMotor, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);

     // Reset Encoder to Zero
    m_encoder.setPosition(kposition.setpoint[0][0]); // Set the current position as the starting position

    // Get the closed loop controllers from the motors
    m_control = m_armtilt.getClosedLoopController();

    // Configure the LOWER LIMIT limit switch.
    m_backwardLimit = new DigitalInput(Tilt.kDIOBaseExtendSwitch);
    // m_forwardLimit = new DigitalInput(Tilt.kDIOBaseRetractSwitch);
  }

  
  /**
   * 
   */
  public void backwards()  // Change name to backwards()
  {
    if (getSwitch()) {
      halt();
    } else {
      m_armtilt.set(-Tilt.kSpeedUp);
    }
  }

  /**
   * 
   */ // change name to forwards()
  public void forwards() { m_armtilt.set(Tilt.kspeedDown); }

  /**
   * 
   */
  public void halt() { m_armtilt.set(0.0); }

  /**
   * 
   * @return
   */ // change to getSwitch()
  public boolean getSwitch() { return !m_backwardLimit.get(); }

  /**
   * 
   */ // change to resetEncoder()
  public void resetEncoder() { m_encoder.setPosition(0.0); }

  /**
   * 
   * @return
   */
  public double getPosition() { return m_encoder.getPosition(); }

  /**
   * 
   * @param posID
   */
  public void setPosition(int posID)
  {
    m_control.setReference(kposition.setpoint[posID][0], ControlType.kPosition);
  }

  /**
   * 
   */
  public void setSmartPosition(int posID)
  {
    m_control.setReference(kposition.setpoint[posID][0], ControlType.kMAXMotionPositionControl);
  }


  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    double position = getPosition();
    boolean proxSwitch_B = getSwitch();
    
    SmartDashboard.putNumber("Arm Tilt Position", position);
    SmartDashboard.putBoolean("Arm Tilt Switch Backward", proxSwitch_B);
    // SmartDashboard.putBoolean("Arm Tilt Switch Forward", proxSwitch_F);

    // Reset the encoder to zero when switch is triggered. Don't just keep resetting the switch because
    // that appears to mess up the motion profile when it can't calculate the velocity. The best way will
    // be to monitor the current and last state of the proxSwitch. If the previous state was false and it
    // just turned true, then zero. If it stays true or stays false or turns from true to false, don't do
    // anything with the zeroing.
    if (proxSwitch_B && !proxSwitch_lastState) { resetEncoder(); halt(); }
    proxSwitch_lastState = proxSwitch_B;
  }
}
