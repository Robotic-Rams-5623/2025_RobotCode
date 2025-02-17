// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

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
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ArmLengthConst;
import frc.robot.Constants.ArmLengthConst.kPositions;


public class ArmExtend extends SubsystemBase {
  /** Creates a new ArmExtend. */
  private final SparkClosedLoopController m_control;
  private final SparkMax m_extend;
  private final SparkMaxConfig m_configmotor;
  private final RelativeEncoder m_encoder;
  private TrapezoidProfile m_profile;
  private TrapezoidProfile.State goal;
  private TrapezoidProfile.State setpoint;
  private Timer m_Timer;
  private double m_setpoint;
  private DigitalInput m_retractlimit;

  public ArmExtend() {
    m_extend = new SparkMax(ArmLengthConst.kIDextend, MotorType.kBrushed);

    

    m_configmotor = new SparkMaxConfig();
    m_configmotor
        .inverted(false)
        .idleMode(IdleMode.kBrake);

    m_extend.configure(m_configmotor, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);

    m_encoder = m_extend.getAlternateEncoder();
    m_encoder.setPosition(0.0);
    
    m_control = m_extend.getClosedLoopController();

    m_Timer = new Timer();
    m_Timer.start();
    m_Timer.reset();
  
    m_retractlimit = new DigitalInput(ArmLengthConst.kDIOextendretractswitch);

    m_setpoint = kPositions.setpoint[0][0];
    
    m_profile = new TrapezoidProfile(ArmLengthConst.kArmMotionConstraint);

    goal = new TrapezoidProfile.State();
    setpoint = new TrapezoidProfile.State();

    
  }

  public void setTargetPosition(double set) {
    if (set !=m_setpoint) {
      m_setpoint = set;
    }
  }

  private void updateMotionprofile(){
    TrapezoidProfile.State state = new TrapezoidProfile.State(m_encoder.getPosition(), m_encoder.getVelocity());
    TrapezoidProfile.State goal = new TrapezoidProfile.State(m_setpoint, 0.0);

    m_Timer.reset();
  }

  public void runAutomatic(){
    double elapsedTime = m_Timer.get();
    if (m_profile.isFinished(elapsedTime)) {
      setpoint = new TrapezoidProfile.State(m_setpoint, 0.0);
    }
    else {
      setpoint =m_profile.calculate(elapsedTime, setpoint, goal);
    }

    m_control.setReference(setpoint.position, ControlType.kPosition);
  }

  public void out(){
    m_extend.set(ArmLengthConst.kSpeedUp);
  }

  public void in(){
    m_extend.set(-ArmLengthConst.kSpeedDown);
  }

  public void stop(){
    m_extend.set(0.0);
  }

  public void resetencoder(){
    m_encoder.setPosition(0);
  }

  public double getPosition() {
    double pos = m_encoder.getPosition();
    return pos;
  }

  public void setArmPosition(int posID){
    m_control.setReference(kPositions.setpoint[posID][0], ControlType.kPosition);
  }

  public boolean getSwitch(){
    return m_retractlimit.get();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("arm extend position", getPosition());
    SmartDashboard.putBoolean("arm extend switch", getSwitch());
  }
}
