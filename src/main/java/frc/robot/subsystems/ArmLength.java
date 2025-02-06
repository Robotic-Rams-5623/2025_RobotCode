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

import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Timer;
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

  private TrapezoidProfile m_Profilebot;
  private TrapezoidProfile m_Profiletop;
  private TrapezoidProfile.State goaltop;
  private TrapezoidProfile.State goalbot;
  private TrapezoidProfile.State setpointtop;
  private TrapezoidProfile.State setpointbot;
  private Timer m_Timer;
  private double m_setpointtop;
  private double m_setpointbot;

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


    m_Timer = new Timer();
    m_Timer.start();
    m_Timer.reset();
    
    m_setpointtop = kPositions.setpoint[0][0];
    m_setpointbot = kPositions.setpoint[0][1];

    m_Profilebot = new TrapezoidProfile(ArmLengthConst.kArmMotionConstraint);
    m_Profiletop = new TrapezoidProfile(ArmLengthConst.kArmMotionConstraint);

    goaltop = new TrapezoidProfile.State();
    goalbot = new TrapezoidProfile.State();
    setpointtop = new TrapezoidProfile.State();
    setpointbot = new TrapezoidProfile.State();

    updateMotionprofile();
  } 

  public void setTargetPosition(double setBot, double setTop) {
    if (setTop != m_setpointtop || setBot != m_setpointbot) {
      m_setpointtop = setTop;
      m_setpointbot = setBot;
      updateMotionprofile();
    }
  }

  private void updateMotionprofile(){
    TrapezoidProfile.State statebot = new TrapezoidProfile.State(m_baseencoder.getPosition(), m_baseencoder.getVelocity());
    TrapezoidProfile.State goalbot = new TrapezoidProfile.State(m_setpointbot, 0.0);
    TrapezoidProfile.State statetop = new TrapezoidProfile.State(m_topEncoder.getPosition(), m_topEncoder.getVelocity());
    TrapezoidProfile.State goaltop = new TrapezoidProfile.State(m_setpointtop, 0.0);

    m_Timer.reset();
  }

  public void runAutomatic() {
      double elapsedTime = m_Timer.get();
      if (m_Profilebot.isFinished(elapsedTime) && m_Profiletop.isFinished(elapsedTime)) {
        setpointtop = new TrapezoidProfile.State(m_setpointtop, 0.0);
        setpointbot = new TrapezoidProfile.State(m_setpointbot, 0.0);
      }
      else {
        setpointtop = m_Profiletop.calculate(elapsedTime, setpointtop, goaltop);
        setpointbot = m_Profilebot.calculate(elapsedTime, setpointbot, goalbot);
      }
  
      // feedforward = Constants.Arm.kArmFeedforward.calculate(m_encoder.getPosition()+Constants.Arm.kArmZeroCosineOffset, targetState.velocity);
      m_basecontrol.setReference(setpointbot.position, ControlType.kPosition);
      m_topcontrol.setReference(setpointtop.position, ControlType.kPosition);
    }
  

  public void baseUp(){
    //m_armbase.set(ArmLengthConstants.kSpeedUp);
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
