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

import frc.robot.Constants.ArmConst.Length;
import frc.robot.Constants.ArmConst.MotorConfigs;
import frc.robot.Constants.CANSignals;
import frc.robot.Constants.ArmConst.kposition;

public class ArmLength extends SubsystemBase {
  /* THIS SECTION CREATES ALL THE EMPTY OBJECTS FOR THIS SUBSYTEM */
  // Create Motor Objects
  private final SparkMax m_armtop;
  // Create Encoder Objects
  // private final RelativeEncoder m_topEncoder;
  private final RelativeEncoder m_topEncoder;
  // Create Closed Loop Controller Objects
  private final SparkClosedLoopController m_topcontrol;
  // Create Motor Configuration Objects
  private final SparkMaxConfig m_configmotor;
  // Create Limit Switch Objects
  private final DigitalInput m_topRetractLimit;

  /* CREATE A NEW ArmLength SUBSYSTEM */
  public ArmLength() {
    /* THIS SECTION ASSIGNS STUFF TO THE CREATED OBJECTS */
    // Define the motors (NEO Brushless plugged into Spark MAX)
    m_armtop = new SparkMax(Length.kIDArmTopLength, MotorType.kBrushless);

    // Define the encoders (Rev Throughbore quadrature encoders plugged into the Alt Encoder Data Port of Spark Max)
    m_topEncoder = m_armtop.getAlternateEncoder();
    
    // Define the motors configuration
    m_configmotor = new SparkMaxConfig();
    m_configmotor
        .inverted(false)
        .idleMode(IdleMode.kBrake)
        .openLoopRampRate(0.1)
        .closedLoopRampRate(0.1);
    m_configmotor.alternateEncoder.apply(MotorConfigs.kAltEncoderConfig_NEO);
    m_configmotor.closedLoop.apply(MotorConfigs.kMotorLoopConfig_Top);
    m_configmotor.softLimit.apply(MotorConfigs.kMotorSoftLimitConfig_Top);
    m_configmotor.signals.apply(CANSignals.ArmMotors.kMotorSignalConfig);
    m_configmotor.closedLoop.maxMotion.apply(MotorConfigs.kMotorSmartMotion_Top);

    // Apply the motor configurations to the motors
    m_armtop.configure(m_configmotor, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);

    // Reset Encoder to Zero
    m_topEncoder.setPosition(kposition.setpoint[0][1]); // Set the current position as zero (Should be home position but can be corrected by hitting retract limit switch 
    
    // Get the closed loop controllers from the motors
    m_topcontrol = m_armtop.getClosedLoopController();

    // Configure the LOWER LIMIT limit switch.
    m_topRetractLimit = new DigitalInput(Length.kDIOTopRetractSwitch);
  } 

  /**
   *
   */
  public void Up(){
    if (gettopswitch()) {
      Halt();
    } else {
      m_armtop.set(Length.kSpeedUp);
    }
  }
  
  /**
   *
   */
  public void Down(){
    m_armtop.set(-Length.kspeedDown);
  }

  /**
   *
   */
  public void Halt(){
    m_armtop.set(0.0);
  }
  
  /**
   *
   */
  public boolean gettopswitch(){
    return !m_topRetractLimit.get();
  }

  /**
   * Reset the encoder of the top linear actuator to its zeroed home position.
   */
  public void resetTopEncoder() {
    m_topEncoder.setPosition(kposition.setpoint[0][1]);
  }

  /**
   * Get the current positions of both encoders.
   * @return Double Array of size two containing base and top encoder positions
   */
  public double getPosition() {
    return m_topEncoder.getPosition();
  }
  
  /**
   * Activate the closed loop control for the arm positions
   * @param posID integer that defines preset positions of the arms
   */
  public void setArmPosition(int posID)
  {
    /**
     * Set the position of both arms simultaneously to move in fluid motion to desired position.
     * A feedforward could be calculated and added to the top control if gravity starts to fight us.
     */
    m_topcontrol.setReference(kposition.setpoint[posID][1], ControlType.kPosition);
  }

  public void setSmartPosition(int posID) {
    m_topcontrol.setReference(kposition.setpoint[posID][1], ControlType.kMAXMotionPositionControl);
  }

  
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    double position = getPosition();
    boolean proxSwitch = gettopswitch();

    SmartDashboard.putNumber("get arm length top", position);
    SmartDashboard.putBoolean("arm top switch", proxSwitch);

    if (proxSwitch) {resetTopEncoder();}
  }
}
