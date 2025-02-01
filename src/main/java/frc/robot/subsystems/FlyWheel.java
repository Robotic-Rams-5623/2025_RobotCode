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

import frc.robot.Constants.FlyWheelConstants;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class FlyWheel extends SubsystemBase {
  // /* Create object containers */
  // private final SparkMax m_flyWheel;
  // private final SparkMaxConfig m_configMotor;
  private final DigitalInput m_coralSwitch;

  /** CREATE A NEW FLYWHEEL **/
  public FlyWheel() {
    // /** Assign objects there classes and parameters **/
    // m_flyWheel = new SparkMax(FlyWheelConstants.kIDFlyWheelMotor, MotorType.kBrushed);
    // m_configMotor = new SparkMaxConfig();
    
    // /** Do the configuration setup for the motor **/
    // m_configMotor
    //     .inverted(false)
    //     .idleMode(IdleMode.kCoast);
    
    // /** Apply the configuration to the motor **/
    // m_flyWheel.configure(m_configMotor, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);

    /* CORAL SWITCH CONFIGURATION */
    m_coralSwitch = new DigitalInput(FlyWheelConstants.kDIOSwitch);
  }

  public void in() {
    // m_flyWheel.set(FlyWheelConstants.kSpeedIn);
  }

  public void out() {
    // m_flyWheel.set(-FlyWheelConstants.kSpeedOut);
  }

  public void stop() {
    // m_flyWheel.set(0.0);
  }

  public boolean getSwitch() {
    return m_coralSwitch.get();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
