// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ArmTiltConstants;

public class ArmTilt extends SubsystemBase {
  /** Creates a new ArmTilt. */
  private final SparkMax m_armtilt;
  private final SparkMaxConfig m_configMotor;

  public ArmTilt() {
    m_armtilt = new SparkMax(ArmTiltConstants.kIDArmTiltMotor, MotorType.kBrushed);
    m_configMotor = new SparkMaxConfig();
    m_configMotor
      .inverted(false)
      .idleMode(IdleMode.kCoast);

    m_armtilt.configure(m_configMotor, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);
  }

  public void up() {
    m_armtilt.set(ArmTiltConstants.kSpeedUp);
  }

  public void down() {
    m_armtilt.set(-ArmTiltConstants.kspeedDown);
  }

  public void stop() {
    m_armtilt.set(0.0); 
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
