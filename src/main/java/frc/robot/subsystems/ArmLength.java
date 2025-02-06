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
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ArmLengthConst;
import frc.robot.Constants.CANSignals;
import frc.robot.Constants.ArmLengthConst.kPositions;
import frc.robot.Constants.ArmLengthConst.kPositions.armSetpoint;

public class ArmLength extends SubsystemBase {
  /* THIS SECTION CREATES ALL THE EMPTY OBJECTS FOR THIS SUBSYTEM */
  // Create Motor Objects
  private final SparkMax m_armbase;
  private final SparkMax m_armtop;
  // Create Encoder Objects
  private final RelativeEncoder m_baseencoder;
  private final RelativeEncoder m_topEncoder;
  // Create Closed Loop Controller Objects
  private final SparkClosedLoopController m_basecontrol;
  private final SparkClosedLoopController m_topcontrol;
  // Create Motor Configuration Objects
  private final SparkMaxConfig m_configmotor;
  // Create Limit Switch Objects
  private final DigitalInput m_topExtendLimit;
  private final DigitalInput m_topRetractLimit;
  private final DigitalInput m_baseExtendLimit;
  private final DigitalInput m_baseRetractLimit;

  /* CREATE A NEW ArmLength SUBSYSTEM */
  public ArmLength() {
    /* THIS SECTION ASSIGNS STUFF TO THE CREATED OBJECTS */

    // Define the motors (NEO Brushless plugged into Spark MAX)
    m_armbase = new SparkMax(ArmLengthConst.kIDArmBaseLength, MotorType.kBrushed);
    m_armtop = new SparkMax(ArmLengthConst.kIDArmTopLength, MotorType.kBrushed);

    // Define the encoders (Rev Throughbore quadrature encoders plugged into the Alt Encoder Data Port of Spark Max)
    m_baseencoder = m_armbase.getAlternateEncoder();
    m_topEncoder = m_armtop.getAlternateEncoder();
    // Set the start positions for the encoders
    m_baseencoder.setPosition(0.0);
    m_topEncoder.setPosition(0.0);

    // Define the motors configuration
    m_configmotor = new SparkMaxConfig();
    m_configmotor
        .inverted(false)
        .idleMode(IdleMode.kBrake)
        .openLoopRampRate(0.1)
        .closedLoopRampRate(0.1);
    m_configmotor.encoder.apply(ArmLengthConst.kMotorEncoderConfig);
    m_configmotor.closedLoop.apply(ArmLengthConst.kMotorLoopConfig);
    m_configmotor.softLimit.apply(ArmLengthConst.kMotorSoftLimitConfig);
    m_configmotor.signals.apply(CANSignals.ArmMotors.kMotorSignalConfig);

    // Apply the motor configurations to the motors
    m_armbase.configure(m_configmotor, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);
    m_armtop.configure(m_configmotor, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);

    // Get the closed loop controllers from the motors
    m_topcontrol = m_armtop.getClosedLoopController();
    m_basecontrol = m_armbase.getClosedLoopController();

    // Configure the limit switches
    m_baseExtendLimit = new DigitalInput(ArmLengthConst.kDIOBaseExtendSwitch);
    m_baseRetractLimit = new DigitalInput(ArmLengthConst.kDIOBaseRetractSwitch);
    m_topExtendLimit = new DigitalInput(ArmLengthConst.kDIOTopExtendSwitch);
    m_topRetractLimit = new DigitalInput(ArmLengthConst.kDIOTopRetractSwitch);
  } 

  public void baseUp(){
    // m_armbase.set(ArmLengthConstants.kSpeedUp);
  }

  public void baseDown(){
    // m_armbase.set(-ArmLengthConstants.kSpeedDown);
  }

  public void baseHalt(){
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

  /**
   * Reset the encoder of the bottom linear actuator to its zeroed home position.
   */
  public void resetBaseEncoder() {
    m_baseencoder.setPosition(0);
  }

  /**
   * Reset the encoder of the top linear actuator to its zeroed home position.
   */
  public void resetTopEncoder() {
    m_topEncoder.setPosition(0.0);
  }

  /**
   * Get the current positions of both encoders.
   * @return Double Array of size two containing base and top encoder positions
   */
  public double[] getPosition() {
    double pos[] = {m_baseencoder.getPosition(), m_topEncoder.getPosition()};
    return pos;
  }
  
  /**
   * Activate the closed loop control for the arm positions
   * @param posID integer that defines preset positions of the arms
   */
  public void setArmPosition(kPositions.armSetpoint posID)
  {
    /**
     * Set the position of both arms simultaneously to move in fluid motion to desired position.
     * A feedforward could be calculated and added to the top control if gravity starts to fight us.
     */
    // m_basecontrol.setReference(kPositions.setpoint[posID][0], ControlType.kPosition);
    // m_topcontrol.setReference(kPositions.setpoint[posID][1], ControlType.kPosition);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
