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
  private final RelativeEncoder m_encoder;
  // Create Closed Loop Controller Objects
  private final SparkClosedLoopController m_control;
  // Create Motor Configuration Objects
  private final SparkMaxConfig m_configMotor;
  // Create Limit Switch Objects
  private final DigitalInput m_downLimit;
  private final DigitalInput m_midLimit;
  private final DigitalInput m_topLimit;

  // Subsystem Variables
  private boolean proxSwitch_lastState;
  private boolean proxSwitch_lastState_M;
  private boolean proxSwitch_lastState_T;
  private boolean proxSwitch_B;
  private boolean proxSwitch_M;
  private boolean proxSwitch_T;
  private double position;

  /* CREATE A NEW ArmLength SUBSYSTEM */
  public ArmLength() {
    /* THIS SECTION ASSIGNS STUFF TO THE CREATED OBJECTS */
    // Define the motors (NEO Brushless plugged into Spark MAX)
    m_armtop = new SparkMax(Length.kIDArmTopLength, MotorType.kBrushless);

    // Define the encoders (Rev Throughbore quadrature encoders plugged into the Alt Encoder Data Port of Spark Max)
    m_encoder = m_armtop.getAlternateEncoder();
    
    // Define the motors configuration
    m_configMotor = new SparkMaxConfig();
    m_configMotor
        .inverted(false)
        .idleMode(IdleMode.kBrake)
        .openLoopRampRate(0.1)
        .closedLoopRampRate(0.1);
    m_configMotor.alternateEncoder.apply(MotorConfigs.kAltEncoderConfig_Top);
    m_configMotor.closedLoop.apply(MotorConfigs.kMotorLoopConfig_Top);
    m_configMotor.softLimit.apply(MotorConfigs.kMotorSoftLimitConfig_Top);
    m_configMotor.signals.apply(CANSignals.ArmMotors.kMotorSignalConfig);
    m_configMotor.closedLoop.maxMotion.apply(MotorConfigs.kMotorSmartMotion_Top);

    // Apply the motor configurations to the motors
    m_armtop.configure(m_configMotor, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);

    // Reset Encoder to Zero
    m_encoder.setPosition(kposition.setpoint[0][1]); // Set the current position as zero (Should be home position but can be corrected by hitting retract limit switch 
    
    // Get the closed loop controllers from the motors
    m_control = m_armtop.getClosedLoopController();

    // Configure the LOWER LIMIT limit switch.
    m_downLimit = new DigitalInput(Length.kDIOHomeSwitch);
    m_midLimit = new DigitalInput(Length.kDIOMidSwitch);
    m_topLimit = new DigitalInput(Length.kDIOTopSwitch);
  } 

  /**
   *
   */ // Change name to up()
  public void Up() {
    if (getTopSwitch()) {
      Halt();
    } else {
      m_armtop.set(Length.kSpeedUp);
    }
  }
    
  
  /**
   *
   */ // Change name to down()
  public void Down(){
    if (getBottomSwitch()) {
      Halt();
    } else {
      m_armtop.set(-Length.kspeedDown);
    }
  }

  /**
   *
   */ // change name to halt()
  public void Halt() { m_armtop.set(0.0); }
  
  /**
   *
   */
  public boolean getBottomSwitch() { return !m_downLimit.get(); }

  public boolean getMidSwitch() { return !m_midLimit.get(); }

  public boolean getTopSwitch() { return !m_topLimit.get(); }

  /**
   * Reset the encoder of the top linear actuator to its zeroed home position.
   */
  public void resetEncoder(double pos) { m_encoder.setPosition(pos); }

  /**
   * Get the current positions of both encoders.
   * @return Double Array of size two containing base and top encoder positions
   */
  public double getPosition() { return m_encoder.getPosition(); }
  
  /**
   * Activate the closed loop control for the arm positions
   * @param posID integer that defines preset positions of the arms
   */
  public void setArmPosition(int posID)
  {
    m_control.setReference(kposition.setpoint[posID][1], ControlType.kPosition);
  }

  public void setSmartPosition(int posID) {
    m_control.setReference(kposition.setpoint[posID][1], ControlType.kMAXMotionPositionControl);
  }

  
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    position = getPosition();
    proxSwitch_B = getBottomSwitch();
    proxSwitch_M = getMidSwitch();
    proxSwitch_T = getTopSwitch();

    SmartDashboard.putNumber("Arm Length Position", position);
    SmartDashboard.putBoolean("Arm Length Switch Down", proxSwitch_B);
    SmartDashboard.putBoolean("Arm Length Switch Middle", proxSwitch_M);
    SmartDashboard.putBoolean("Arm Length Switch Up", proxSwitch_T);

    if (proxSwitch_B && !proxSwitch_lastState) { resetEncoder(kposition.setpoint[0][1]); }
    if (proxSwitch_M && !proxSwitch_lastState_M) { resetEncoder(4.0); }

    
    proxSwitch_lastState = proxSwitch_B;
    proxSwitch_lastState_M = proxSwitch_M;
  }
}
