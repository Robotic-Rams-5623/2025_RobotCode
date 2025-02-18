// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.lang.module.FindException;

import com.revrobotics.spark.config.ClosedLoopConfig;
import com.revrobotics.spark.config.EncoderConfig;
import com.revrobotics.spark.config.SignalsConfig;
import com.revrobotics.spark.config.SoftLimitConfig;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;

import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import swervelib.math.Matter;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean constants. This
 * class should not be used for any other purpose. All constants should be declared globally (i.e. public static). Do
 * not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants
{

  public static final class FlyWheelConstants
  {
    public static final int kIDFlyWheelMotor = 27;
    public static final double kSpeedIn = 0.5;
    public static final double kSpeedOut = 0.5;
    public static final int kDIOSwitch = 0;
  }



  public static final class HandTiltConstants
  {
    public static final class Tilt {
      public static final int kIDHandTiltMotor = 26;
      public static final double kSpeedUp = 0.5;
      public static final double kSpeedDown = 0.5;
      public static final int kDIOtiltdownswitch = 1;

      public static final double kLoopRange[] = {-.2, .2};
      public static final double kIzone = 5; // DERGREES
      public static final double kPIDF[] = {.001, 0, 0, 100};
      public static final double kPosConversion = 0.1607200257;
      public static final double kVelConversion = 1000; 
      public static final int kCPR = 2240;
      public static final double ktiltLimit = 150.0;
    
      // TILT MOTOR CONFIGURATION SUB-COMPONENTS (FOR CLEANER CODE)
      public static final ClosedLoopConfig kTiltLoopConfig = new ClosedLoopConfig()
                .feedbackSensor(FeedbackSensor.kAlternateOrExternalEncoder)
                .outputRange(kLoopRange[0], kLoopRange[1])
                .iZone(kIzone)
                .positionWrappingEnabled(false)
                .pidf(kPIDF[0], kPIDF[1], kPIDF[2], kPIDF[3]);
      public static final EncoderConfig kTiltEncoderConfig = new EncoderConfig()
                .positionConversionFactor(kPosConversion)
                .velocityConversionFactor(kVelConversion)
                .inverted(false)
                .countsPerRevolution(kCPR);
      public static final SoftLimitConfig kTiltSoftLimitConfig = new SoftLimitConfig()
                .forwardSoftLimit(ktiltLimit)
                .forwardSoftLimitEnabled(false)
                .reverseSoftLimit(0.0)
                .reverseSoftLimitEnabled(false);
    }

    public static final class Grab {
      // GRABBER CONFIG
      public static final int kIDGrabbyThingy = 24;
      public static final int kIDGrabbyThingy2 = 25;

      public static final double kSpeedUp = 0.5;
      public static final double kSpeedDown = 0.5;
    }
  }


  public static final class ArmConst
  {
    /* ARM POSITION SETPOINT CONSTANTS */
    public static final class kposition
    {
    /*
     * Set the reference positions for the arm closed loop control.
     * The 1st element in the array will correspond to the ID of
     * the desired position. The second int parameter in the array
     * corresponds to one of three reference positions for one of
     * three closed loop controlers, one for each motor in the system.
     * [0][] - Home Position. Will correspond with reverse limit switches.
     * [1][] - Position 1 - Floor Coral Pickup
     * [2][] - Position 2 - Human Player Coral Pickup
     * [3][] - Position 3 - Base Reef
     * [4][] - Position 4 - Low Reef
     * [5][] - Position 5 - Mid Reef
     * [6][] - Position 6 - Top Reef
     * [7][] - Position 7 - Floor Algea
     * [8][] - Position 8 - Lower Algea Reef
     * [9][] - Position 9 - Upper Algea Reed
     * [][] - Position  - Barge Height
     * [][] - Position  - Climb Height
     * [][] - Position  - MAX Possible Extension
     */
      public static final double setpoint[][] = 
      {
        // {BOTTOM INCHES, TOP INCHES, EXTENSION INCHES, HAND TILT ANGLE DEGREES}
        {0,       0,      0,      0},       // HOME POSITION
        {1,       1,      1,      0},       // POSITION 1
        {3,       2,      0,      0},       // POSITION 2
        {3,       3,      0,      0},       // POSITION 3
        {3,       4,      0,      0},       // POSITION 4
        {2,       5,      4,      0},       // POSITION 5
        {2,       5,      4,      0},       // POSITION 6
        {2,       5,      4,      0},       // POSITION 7
        {2,       5,      4,      0},       // POSITION 8
        {2,       5,      4,      0},       // POSITION 9
        {2,       5,      4,      0},       // POSITION 10
        {2,       5,      4,      0},       // POSITION 11
        {2,       5,      4,      0},       // POSITION 12
      };
    }
    
    /* TILT ARM CONSTANTS */
    public static final class Tilt
    {
      // IDs
      public static final int kIDArmTiltMotor = 22; // CAN ID
      public static final int kDIOBaseExtendSwitch = 4;     // RoboRIO DIO Port Number
      // Speeds
      public static final double kSpeedUp = 0.5;    // Manual Speed to Tilt Arm Mechanism Backwards
      public static final double kspeedDown = 0.5;  // Manual Speed to Tilt Arm Mechanism Forwards (ONLY FOR CLIMBING/RESET)
      // Trapezoid Profile Constant
      public static final TrapezoidProfile.Constraints kArmMotionConstraint = new TrapezoidProfile.Constraints(2.0, 2.0);
      // Closed Loop Control
      public static final double[] kLoopRange = {-0.5,0.6}; // Allowable %Output of Closed Loop Controller (i.e. can't go faster then ±60%)
      public static final double[] kPIDF = {0.01, 0, 0, 0};   // {P, I, D, FF} Closed Loop Constants (F = 1/Kv from motor spec sheet if using velocity control, otherwise SET TO ZERO)
      public static final double kIzone = 5;              // Integral Constant Zone
    }
    
    /* LENGTH ARM CONSTANTS */
    public static final class Length
    {
      // IDs
      public static final int kIDArmTopLength = 23; // CAN ID
      public static final int kDIOTopRetractSwitch = 3;     // RoboRIO DIO Port Number
      // Speeds
      public static final double kSpeedUp = 0.5;    // Manual Speed to Lift Upper Arm Mechanism
      public static final double kspeedDown = 0.5;  // Manual Speed to Lower Upper Arm Mechanism
      // Trapezoid Profile Constant
      public static final TrapezoidProfile.Constraints kArmMotionConstraint = new TrapezoidProfile.Constraints(2.0, 2.0);
    }
    
    /* EXTEND ARM CONSTANTS */
    public static final class Extend
    {
      // IDs
      public static final int kIDextend = 28;
      public static final int kDIOextendretractswitch = 2;
      // Speeds
      public static final double kSpeedUp = 0.5;    // Manual Speed to Extend Upper Arm Mechanism
      public static final double kspeedDown = 0.5;  // Manual Speed to Retract Upper Arm Mechanism
    }

    /* ARM MOTOR CONFIGURATIONS */
    public static final class MotorConfigs
    {
      // ENCODER CONFIG CONSTANTS
      public static final double kPosConversion_NEO = 0.5;    // (Default output is rotations) Inch = Rotation * PosConversion = Rot * (0.5 inch / 1 rot)
      public static final double kVelConversion_NEO = 0.5 / 60;    // (Default output is RPM) Inch/Sec = RPM * VelConversion = RPM * (0.5 inch / 1 rev) * (1 min/ 60 sec)
      public static final double kPosConversion_HD = 1.0;    // (Default output is rotations) Inch = Rotation * PosConversion = Rot * ~~~~
      public static final double kVelConversion_HD = 1.0;    // (Default output is RPM) Inch/Sec = RPM * VelConversion = RPM * ~~~~
      public static final int kCPR_RevBore = 8192;                // Encoder counts per revolution (Rev Throughbore = 8192)
      public static final int kCPR_HD = 28;                // Encoder counts per revolution (HD Built-in Encoder AT MOTOR SHAFT = 28) // Doesnt include gear box
      // MOTOR SOFT LIMITS
      public static final double kbaseExtendLimit = 4.0;  // Soft limit in inches as read from encoder
      public static final double ktopRetractLimit = 4.0;  // Soft limit in inches as read from encoder
      public static final double kextendExtendLimit = 5.0; // Soft limit in inches as read from encoder
      // CLOSED LOOP CONSTANTS
      public static final double[] kLoopRange_NEO = {-0.5,0.6}; // Allowable %Output of Closed Loop Controller (i.e. can't go faster then ±60%)
      public static final double[] kPIDF_NEO = {0.01, 0, 0, 0};   // {P, I, D, FF} Closed Loop Constants (F = 1/Kv from motor spec sheet if using velocity control, otherwise SET TO ZERO)
      public static final double kIzone_NEO = 5;              // Integral Constant Zone
      public static final double[] kLoopRange_HD = {-0.5,0.6}; // Allowable %Output of Closed Loop Controller (i.e. can't go faster then ±60%)
      public static final double[] kPIDF_HD = {0.01, 0, 0, 0};   // {P, I, D, FF} Closed Loop Constants (F = 1/Kv from motor spec sheet if using velocity control, otherwise SET TO ZERO)
      public static final double kIzone_HD = 5;              // Integral Constant Zone

      // ALTERNATE ENCODER SPARK MAX CONFIGURATIONS
      public static final AlternateEncoderConfig kAltEncoderConfig_NEO = new AlternateEncoderConfig() // MAY HAVE TO USE setSparkMaxDataPortConfig() method to configure the data port for alternate sensor mode.
          .positionConversionFactor(kPosConversion_NEO) // 
          .velocityConversionFactor(kVelConversion_NEO) // Default output is RPM.
          .inverted(false) // Positive motor direction should equal positive encoder movement.
          .countsPerRevolution(kCPR_RevBore) // Encoder counts per revolution using Through Bore Encoder
          .measurementPeriod(10) // period in ms of the position measurement used for calculating the velocity. (Change in Position in Period)/(Period) = Velocity
          .averageDepth(64); // Default = 64 ~ Number of samples averaged for velocity reading, quadratures can be 1 to 64. 64 measurements average into one velocity measurement.
      public static final AlternateEncoderConfig kAltEncoderConfig_HD = new AlternateEncoderConfig() // MAY HAVE TO USE setSparkMaxDataPortConfig() method to configure the data port for alternate sensor mode.
          .positionConversionFactor(kPosConversion_HD) // 
          .velocityConversionFactor(kVelConversion_HD) // Default output is RPM.
          .inverted(false) // Positive motor direction should equal positive encoder movement.
          .countsPerRevolution(kCPR_HD) // Encoder counts per revolution using Through Bore Encoder
          .measurementPeriod(10) // period in ms of the position measurement used for calculating the velocity. (Change in Position in Period)/(Period) = Velocity
          .averageDepth(32); // Default = 64 ~ Number of samples averaged for velocity reading, quadratures can be 1 to 64. 64 measurements average into one velocity measurement.
      
      // CLOSED LOOP SPARK MAX CONFIGURATIONS
      public static final ClosedLoopConfig kMotorLoopConfig_NEO = new ClosedLoopConfig()
          .feedbackSensor(FeedbackSensor.kAlternateOrExternalEncoder)
          .outputRange(kLoopRange_NEO[0], kLoopRange_NEO[1])
          .iZone(kIzone_NEO)
          .positionWrappingEnabled(false)
          .pidf(kPIDF_NEO[0], kPIDF_NEO[1], kPIDF_NEO[2], kPIDF_NEO[3]);
      public static final ClosedLoopConfig kMotorLoopConfig_HD = new ClosedLoopConfig()
          .feedbackSensor(FeedbackSensor.kAlternateOrExternalEncoder)
          .outputRange(kLoopRange_HD[0], kLoopRange_HD[1])
          .iZone(kIzone_HD)
          .positionWrappingEnabled(false)
          .pidf(kPIDF_HD[0], kPIDF_HD[1], kPIDF_HD[2], kPIDF_HD[3]);
      
      // SOFT LIMIT SPARK MAX CONFIGURATIONS
      public static final SoftLimitConfig kMotorSoftLimitConfig_Base = new SoftLimitConfig()
          .forwardSoftLimit(kbaseExtendLimit)
          .forwardSoftLimitEnabled(true)
          .reverseSoftLimit(0.0)
          .reverseSoftLimitEnabled(false);
      public static final SoftLimitConfig kMotorSoftLimitConfig_Top = new SoftLimitConfig()
          .forwardSoftLimit(0.0)
          .forwardSoftLimitEnabled(false)
          .reverseSoftLimit(ktopRetractLimit)
          .reverseSoftLimitEnabled(true);
      public static final SoftLimitConfig kMotorSoftLimitConfig_Extend = new SoftLimitConfig()
          .forwardSoftLimit(kextendExtendLimit)
          .forwardSoftLimitEnabled(true)
          .reverseSoftLimit(0.0)
          .reverseSoftLimitEnabled(false);
    }

    
  }





  
  public static final class DrivebaseConstants
  {
    // Hold time on motor brakes when disabled
    public static final double WHEEL_LOCK_TIME = 10; // seconds
    public static final double ROBOT_MASS = (46) * 0.453592; // 32lbs * kg per pound
    public static final Matter CHASSIS    = new Matter(new Translation3d(0, 0, Units.inchesToMeters(8)), ROBOT_MASS);
    public static final double LOOP_TIME  = 0.13; //s, 20ms + 110ms sprk max velocity lag
    public static final double MAX_SPEED  = Units.feetToMeters(5); // Max speed of 2024 Chassis is ~5 m/s
    // Maximum speed of the robot in meters per second, used to limit acceleration.
  }




  public static class OperatorConstants
  {

    // Joystick Deadband
    public static final double DEADBAND        = 0.1;
    public static final double LEFT_Y_DEADBAND = 0.1;
    public static final double RIGHT_X_DEADBAND = 0.1;
    public static final double TURN_CONSTANT    = 3;
  }

  //  public static final class AutonConstants
//  {
//    public static final PIDConstants TRANSLATION_PID = new PIDConstants(0.7, 0, 0);
//    public static final PIDConstants ANGLE_PID       = new PIDConstants(0.4, 0, 0.01);
//  }





  
  public static final class CANSignals
  {
    public static final class ArmMotors
    {
      public static final SignalsConfig kMotorSignalConfig = new SignalsConfig()
            .analogPositionAlwaysOn(false)
            .analogVelocityAlwaysOn(false)
            .analogVoltageAlwaysOn(false)
            .analogPositionPeriodMs(0)
            .analogVelocityPeriodMs(0)
            .analogVoltagePeriodMs(0)

            .absoluteEncoderPositionAlwaysOn(false)
            .absoluteEncoderVelocityAlwaysOn(false)
            .externalOrAltEncoderPositionAlwaysOn(true)
            .externalOrAltEncoderVelocityAlwaysOn(true)
            .absoluteEncoderPositionPeriodMs(0)
            .absoluteEncoderVelocityPeriodMs(0)
            .externalOrAltEncoderPosition(10)
            .externalOrAltEncoderVelocity(10)

            .primaryEncoderPositionAlwaysOn(false)
            .primaryEncoderVelocityAlwaysOn(false)
            .primaryEncoderPositionPeriodMs(0)
            .primaryEncoderVelocityPeriodMs(0)

            .iAccumulationAlwaysOn(false)
            .iAccumulationPeriodMs(0)

            .faultsAlwaysOn(false)
            .warningsAlwaysOn(true)
            .faultsPeriodMs(0)
            .warningsPeriodMs(500)

            .appliedOutputPeriodMs(0)
            .busVoltagePeriodMs(0)
            .limitsPeriodMs(10)
            .motorTemperaturePeriodMs(0)
            .outputCurrentPeriodMs(0);   
    }
    public static final class HandMotors
    {
      public static final SignalsConfig kMotorSignalConfig_Dumb = new SignalsConfig()
            .analogPositionAlwaysOn(false)
            .analogVelocityAlwaysOn(false)
            .analogVoltageAlwaysOn(false)
            .analogPositionPeriodMs(0)
            .analogVelocityPeriodMs(0)
            .analogVoltagePeriodMs(0)

            .absoluteEncoderPositionAlwaysOn(false)
            .absoluteEncoderVelocityAlwaysOn(false)
            .externalOrAltEncoderPositionAlwaysOn(false)
            .externalOrAltEncoderVelocityAlwaysOn(false)
            .absoluteEncoderPositionPeriodMs(0)
            .absoluteEncoderVelocityPeriodMs(0)
            .externalOrAltEncoderPosition(0)
            .externalOrAltEncoderVelocity(0)

            .primaryEncoderPositionAlwaysOn(false)
            .primaryEncoderVelocityAlwaysOn(false)
            .primaryEncoderPositionPeriodMs(0)
            .primaryEncoderVelocityPeriodMs(0)

            .iAccumulationAlwaysOn(false)
            .iAccumulationPeriodMs(0)

            .faultsAlwaysOn(false)
            .warningsAlwaysOn(true)
            .faultsPeriodMs(0)
            .warningsPeriodMs(500)

            .appliedOutputPeriodMs(0)
            .busVoltagePeriodMs(0)
            .limitsPeriodMs(0)
            .motorTemperaturePeriodMs(0)
            .outputCurrentPeriodMs(0);
      public static final SignalsConfig kMotorSignalConfig_Tilt = new SignalsConfig()
            .analogPositionAlwaysOn(false)
            .analogVelocityAlwaysOn(false)
            .analogVoltageAlwaysOn(false)
            .analogPositionPeriodMs(0)
            .analogVelocityPeriodMs(0)
            .analogVoltagePeriodMs(0)

            .absoluteEncoderPositionAlwaysOn(false)
            .absoluteEncoderVelocityAlwaysOn(false)
            .externalOrAltEncoderPositionAlwaysOn(false)
            .externalOrAltEncoderVelocityAlwaysOn(false)
            .absoluteEncoderPositionPeriodMs(0)
            .absoluteEncoderVelocityPeriodMs(0)
            .externalOrAltEncoderPosition(0)
            .externalOrAltEncoderVelocity(0)

            .primaryEncoderPositionAlwaysOn(true)
            .primaryEncoderVelocityAlwaysOn(true)
            .primaryEncoderPositionPeriodMs(10)
            .primaryEncoderVelocityPeriodMs(10)

            .iAccumulationAlwaysOn(false)
            .iAccumulationPeriodMs(0)

            .faultsAlwaysOn(false)
            .warningsAlwaysOn(true)
            .faultsPeriodMs(0)
            .warningsPeriodMs(500)

            .appliedOutputPeriodMs(0)
            .busVoltagePeriodMs(0)
            .limitsPeriodMs(10)
            .motorTemperaturePeriodMs(0)
            .outputCurrentPeriodMs(0);
    }
  }
}
