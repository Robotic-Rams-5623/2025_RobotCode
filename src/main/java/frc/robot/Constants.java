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
    public static final int kIDFlyWheelMotor = 20;
    public static final double kSpeedIn = 0.5;
    public static final double kSpeedOut = 0.5;
    public static final int kDIOSwitch = 0;
  }



  public static final class HandTiltConstants
  {
    public static final class Tilt {
      public static final int kIDHandTiltMotor = 21;
      public static final double kSpeedUp = 0.5;
      public static final double kSpeedDown = 0.5;

      public static final double kLoopRange[] = {-.2, .2};
      public static final double kIzone = 5; // DERGREES
      public static final double kPIDF[] = {.001, 0, 0, 100};
      public static final double kPosConversion = 0.1607200257;
      public static final double kVelConversion = 1000; 
      public static final int kCPR = 2240;
      public static final double ktiltLimit = 180.0;
    
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



  public static final class ArmTiltConstants
  {
    public static final int kIDArmTiltMotor = 22;
    public static final double kSpeedUp = 0.5;
    public static final double kspeedDown = 0.5;


  }



  public static final class ArmLengthConst
  {
    public static final int kIDextend = 28;
    public static final int kIDArmBaseLength = 22;        // CAN Bus ID Number
    public static final int kIDArmTopLength = 23;         // CAN Bus ID Number
    public static final int kDIOBaseRetractSwitch = 1;    // RoboRIO DIO Port Number
    public static final int kDIOBaseExtendSwitch = 2;     // RoboRIO DIO Port Number
    public static final int kDIOTopRetractSwitch = 3;     // RoboRIO DIO Port Number
    public static final int kDIOTopExtendSwitch = 4;      // RoboRIO DIO Port Number
    public static final TrapezoidProfile.Constraints kArmMotionConstraint = new TrapezoidProfile.Constraints(2.0, 2.0);

    /* SET SPEED LIMITATIONS */
    public static final double kSpeedUp = 0.5;            // Manual Extend Speed
    public static final double kSpeedDown = 0.5;          // Manual Retract Speed
    public static final double[] kLoopRange = {-0.5,0.6}; // Allowable %Output of Closed Loop Controller (i.e. can't go faster then ±60%)

    /* CLOSED LOOP CONTROL CONSTANTS */
    public static final double[] kPIDF = {0.01, 0, 0, 0};   // {P, I, D, FF} Closed Loop Constants (F = 1/Kv from motor spec sheet if using velocity control, otherwise SET TO ZERO)
    public static final double kIzone = 5;              // Integral Constant Zone
    public static final double kPosConversion = 0.5/8192;    // Output of encoder * position factor = output in inches (0.5" = 8192 counts)
    public static final double kVelConversion = 1000;   // 
    public static final int kCPR = 8192;                // Encoder counts per revolution (Rev Throughbore = 8192)

    /* ARM LIMITS */
    public static final double kbaseExtendLimit = 6.0;  // Soft limit in inches as read from encoder
    public static final double ktopExtendLimit = 6.0;  // Soft limit in inches as read from encoder

    // MOTOR CONFIGURATION SUB-COMPONENTS (FOR CLEANER CODE)
    public static final ClosedLoopConfig kMotorLoopConfig = new ClosedLoopConfig()
              .feedbackSensor(FeedbackSensor.kAlternateOrExternalEncoder)
              .outputRange(kLoopRange[0], kLoopRange[1])
              .iZone(kIzone)
              .positionWrappingEnabled(false)
              .pidf(kPIDF[0], kPIDF[1], kPIDF[2], kPIDF[3]);
    public static final EncoderConfig kMotorEncoderConfig = new EncoderConfig()
              .positionConversionFactor(kPosConversion)
              .velocityConversionFactor(kVelConversion)
              .inverted(false)
              .countsPerRevolution(kCPR);
    public static final SoftLimitConfig kMotorSoftLimitConfig = new SoftLimitConfig()
              .forwardSoftLimit(kbaseExtendLimit)
              .forwardSoftLimitEnabled(false)
              .reverseSoftLimit(0.0)
              .reverseSoftLimitEnabled(false);

    /*
     * Set the reference positions for the arm closed loop control.
     * The 1st element in the array will correspond to the ID of
     * the desired position. The second int parameter in the array
     * corresponds to one of three reference positions for one of
     * three closed loop controlers, one for each motor in the system.
     * [0][] - Home Position. Will correspond with reverse limit switches.
     * [1][] - Position 1 - Floor Pickup
     * [2][] - Position 2 - Human Player Pickup
     * [3][] - Position 3 - Low Reef
     * [4][] - Position 4 - Mid Reef
     * [5][] - Position 5 - Top Reef
     * [6][] - Position 6 - Barge Height
     * [7][] - Position 7 - Climg Height
     * [8][] - Position 8 - MAX Possible Extension
     */
    public static final class kPositions
    {
      public static enum armSetpoint {
        HOME,
        FLOOR_PICKUP,
        HP_PICKUP,
        REEF1,
        REEF2,
        REEF3,
        BARGE,
        CLIMB,
        MAX
      }

      public static final double setpoint[][] = 
      {
        // {BOTTOM POSITION, TOP POSITION, EXTENSION POSITION} // UNITS IN INCHES
        {0,       0,      0},       // HOME POSITION
        {3,       1,      0},       // POSITION 1
        {3,       2,      0},       // POSITION 2
        {3,       3,      0},       // POSITION 3
        {3,       4,      0},       // POSITION 4
        {2,       5,      4},       // POSITION 5
      };
    }    
  }






  public static final class DrivebaseConstants
  {
    // Hold time on motor brakes when disabled
    public static final double WHEEL_LOCK_TIME = 10; // seconds
    public static final double ROBOT_MASS = (46) * 0.453592; // 32lbs * kg per pound
    public static final Matter CHASSIS    = new Matter(new Translation3d(0, 0, Units.inchesToMeters(8)), ROBOT_MASS);
    public static final double LOOP_TIME  = 0.13; //s, 20ms + 110ms sprk max velocity lag
    public static final double MAX_SPEED  = Units.feetToMeters(5);
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
            .externalOrAltEncoderPosition(100)
            .externalOrAltEncoderVelocity(200)

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
            .primaryEncoderPositionPeriodMs(100)
            .primaryEncoderVelocityPeriodMs(200)

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
    }
  }
}
