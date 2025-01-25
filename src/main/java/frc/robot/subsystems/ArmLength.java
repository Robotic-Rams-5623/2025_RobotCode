// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.servohub.ServoHub.ResetMode;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ArmLengthConstants;

public class ArmLength extends SubsystemBase {
  /** Creates a new ArmLength. */
  private final SparkMax m_armbase;
  private final SparkMax m_armtop;
  private final SparkMaxConfig m_configmotor;

  public ArmLength() {
    m_armbase = new SparkMax(ArmLengthConstants.kIDArmBaseLength, MotorType.kBrushed);
    m_armtop = new SparkMax(ArmLengthConstants.kIDArmTopLength, MotorType.kBrushed);
    
    m_configmotor = new SparkMaxConfig();
    m_configmotor
      .inverted(false)
      .idleMode(IdleMode.kCoast);

    m_armbase.configure(m_configmotor, null, null);
    m_armtop.configure(m_configmotor, null, null);
  } 

  public void in(){
    m_armbase.set(ArmLengthConstants.kSpeedUp);
  }

  public void out(){
    m_armbase.set(-ArmLengthConstants.kSpeedDown);
  }

  public void stop(){
    m_armbase.set(0.0);
  }

  public void up(){
    m_armtop.set(ArmLengthConstants.kSpeedUp);
  }

  public void down(){
    m_armtop.set(-ArmLengthConstants.kSpeedDown);
  }

  public void halt(){
    m_armtop.set(0.0);
  }
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
