package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import frc.robot.Constants.FlyWheelConstants;
import frc.robot.Constants.CANSignals;


public class FlyWheel extends SubsystemBase {
  /* THIS SECTION CREATES ALL THE EMPTY OBJECTS FOR THIS SUBSYTEM */
  // Create Motor Objects
  private final SparkMax m_flyWheel;
  // Create Motor Configuration Objects
  private final SparkMaxConfig m_configMotor;
  // Create Limit Switch Objects
  private final DigitalInput m_coralSwitch;

  /** CREATE A NEW FLYWHEEL **/
  public FlyWheel() {
    /* THIS SECTION ASSIGNS STUFF TO THE CREATED OBJECTS */
    // Define the motors (Rev HD Hex Brushed plugged into Spark MAX)
    m_flyWheel = new SparkMax(FlyWheelConstants.kIDFlyWheelMotor, MotorType.kBrushed);

    // Define the motors configuration
    m_configMotor = new SparkMaxConfig();
    m_configMotor
        .inverted(true)
        .idleMode(IdleMode.kCoast);
    m_configMotor.signals.apply(CANSignals.HandMotors.kMotorSignalConfig_Dumb);
    
    // Apply the motor configurations to the motors
    m_flyWheel.configure(m_configMotor, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);

    // Configure the LOWER LIMIT limit switch.
    m_coralSwitch = new DigitalInput(FlyWheelConstants.kDIOSwitch);
  }

  public void in() {
    if (getSwitch()) {
      stop();
    } else {
      m_flyWheel.set(FlyWheelConstants.kSpeedIn);
    }
    
  }

  public void out() {
    m_flyWheel.set(-FlyWheelConstants.kSpeedOut);
  }

  public void stop() {
    m_flyWheel.set(0.0);
  }

  public boolean getSwitch() {
    return !m_coralSwitch.get();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putBoolean("Coral In?", getSwitch());
  }
}
