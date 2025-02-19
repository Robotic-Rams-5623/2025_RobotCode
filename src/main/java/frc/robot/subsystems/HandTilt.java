package frc.robot.subsystems;

import com.revrobotics.spark.SparkLowLevel.MotorType;

import com.revrobotics.spark.SparkMaxAlternateEncoder;
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
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.math.trajectory.TrapezoidProfile;

import frc.robot.Constants.HandTiltConstants.Tilt;
import frc.robot.Constants.HandTiltConstants.Grab;
import frc.robot.Constants.CANSignals;
import frc.robot.Constants.HandTiltConstants;

public class HandTilt extends SubsystemBase {
  /* THIS SECTION CREATES ALL THE EMPTY OBJECTS FOR THIS SUBSYTEM */
  // Create Motor Objects
  private final SparkMax m_handtilt;
  private final SparkMax m_grableft;
  private final SparkMax m_grabright;
  // Create Encoder Objects
  // private final RelativeEncoder m_tiltencoder;
  private final RelativeEncoder m_tiltencoder;
  // Create Closed Loop Controller Objects
  private final SparkClosedLoopController m_tiltcontrol;
  // Create Motor Configuration Objects
  private final SparkMaxConfig m_configMotor;
  private final SparkMaxConfig m_configgrableft;
  private final SparkMaxConfig m_configgrabright;
  // Create Limit Switch Objects
  private final DigitalInput m_tiltlimit;
  // Create Trapezoidal closed loop profile Objects
  // private TrapezoidProfile m_Profiletop;
  // private TrapezoidProfile.State goaltop;
  // private TrapezoidProfile.State setpointtop;
  // private Timer m_Timer;
  // private double m_setpointtop;

  /* CREATE A NEW HandTilt SUBSYSTEM */
  public HandTilt() {
    /* THIS SECTION ASSIGNS STUFF TO THE CREATED OBJECTS */
    // Define the motors (Rev HD Hex Brushed plugged into Spark MAX)
    m_handtilt = new SparkMax(Tilt.kIDHandTiltMotor, MotorType.kBrushed);
    m_grableft = new SparkMax(Grab.kIDGrabbyThingy, MotorType.kBrushed);
    m_grabright = new SparkMax(Grab.kIDGrabbyThingy2, MotorType.kBrushed);

    // Define the encoders (Rev Throughbore quadrature encoders plugged into the Alt Encoder Data Port of Spark Max)
    m_tiltencoder = m_handtilt.getAlternateEncoder();

    // Define the motors configuration
    m_configMotor = new SparkMaxConfig();
    m_configMotor
        .inverted(true)
        .idleMode(IdleMode.kBrake);
    m_configMotor.alternateEncoder.apply(Tilt.kTiltEncoderConfig);
    m_configMotor.closedLoop.apply(Tilt.kTiltLoopConfig);
    m_configMotor.softLimit.apply(Tilt.kTiltSoftLimitConfig);
    m_configMotor.signals.apply(CANSignals.HandMotors.kMotorSignalConfig_Tilt);
    
    m_configgrableft = new SparkMaxConfig();
    m_configgrableft
        .inverted(false)
        .idleMode(IdleMode.kBrake);
    m_configgrableft.signals.apply(CANSignals.HandMotors.kMotorSignalConfig_Dumb);
    //m_configgrableft.softLimit.apply(); // Set current limiting

    m_configgrabright = new SparkMaxConfig();
    m_configgrabright
        .idleMode(IdleMode.kBrake)
        .follow(m_grableft, true); // Follow the settings of the other motor.

    // Apply the motor configurations to the motors
    m_handtilt.configure(m_configMotor, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);
    m_grableft.configure(m_configgrableft, ResetMode.kNoResetSafeParameters,  PersistMode.kPersistParameters);
    m_grabright.configure(m_configgrabright, ResetMode.kNoResetSafeParameters,  PersistMode.kPersistParameters);

    // Reset Encoder to Zero

    // Get the closed loop controllers from the motors
    m_tiltcontrol = m_handtilt.getClosedLoopController();

    // Configure the LOWER LIMIT limit switch.
    m_tiltlimit = new DigitalInput(Tilt.kDIOtiltdownswitch);

    // Configure the items needed for trapezoidal profiling
    // m_Timer = new Timer();
    // m_Timer.start();
    // m_Timer.reset();
    
    // m_setpoint = kposition.setpoint[0][3];

    // m_Profile = new TrapezoidProfile(Tilt.kArmMotionConstraint);
    // goal = new TrapezoidProfile.State();
    // setpoint = new TrapezoidProfile.State();

    // Update the current motion profile with the current position.
    // updateMotionprofile();
  }

  // public void setTargetPosition(double set) {
  //   if (set != m_setpoint) {
  //     m_setpoint = set;
  //     updateMotionprofile();
  //   }
  // }

  // private void updateMotionprofile(){
  //   TrapezoidProfile.State statetop = new TrapezoidProfile.State(getangle(), getspeed());
  //   TrapezoidProfile.State goaltop = new TrapezoidProfile.State(m_setpoint, 0.0);

  //   m_Timer.reset();
  // }

  // public void runAutomatic() {
  //     double elapsedTime = m_Timer.get();
  //     if (m_Profile.isFinished(elapsedTime)) {
  //       setpoint = new TrapezoidProfile.State(m_setpoint, 0.0);
  //     }
  //     else {
  //       setpoint = m_Profile.calculate(elapsedTime, setpoint, goal);
  //     }
  
  //     // feedforward = Constants.Arm.kArmFeedforward.calculate(m_encoder.getPosition()+Constants.Arm.kArmZeroCosineOffset, targetState.velocity);
  //     m_tiltcontrol.setReference(setpoint.position, ControlType.kPosition);
  //   }

  public void up() {
    if (getswitch()) {
      stop();
    } else {
      m_handtilt.set(Tilt.kSpeedUp);
    }
  }

  public void down() {
    m_handtilt.set(-Tilt.kSpeedDown);
  }

  public void stop() {
    m_handtilt.set(0.0);
  }

  // public void setAngle(double angle) {
  //   m_tiltcontrol.setReference(angle, ControlType.kPosition);
  // }

  public void open() {
    m_grableft.set(Grab.kSpeedUp);
  }
  
  public void close() {
    m_grableft.set(-Grab.kSpeedDown);
  }

  public void halt() {
    m_grableft.set(0.0);
  }

  public double getangle(){
    return m_tiltencoder.getPosition();
  }

  public double getspeed() {
    return m_tiltencoder.getVelocity();
  }

  public boolean getswitch(){
    return !m_tiltlimit.get(); // Lower tilt limit
  }

  public void resetAngle() {
    m_tiltencoder.setPosition(0.0);
  }


  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    boolean switchState = getswitch();
    double motorAngle = getangle();
    
    SmartDashboard.putNumber("Tilt Angle", motorAngle);
    SmartDashboard.putBoolean("Tilt Dow Limit", switchState);

    // If the switch is hit and the angle isnt too far off, reset the encoder to zero.
    if (switchState ) {resetAngle();} //&& ((motorAngle <= 10) && (motorAngle >= -10))
  }
}
