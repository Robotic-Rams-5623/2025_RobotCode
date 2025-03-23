// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.revrobotics.spark.config.ClosedLoopConfig;
import com.revrobotics.spark.config.MAXMotionConfig;
import com.revrobotics.spark.config.AlternateEncoderConfig;
import com.revrobotics.spark.config.SignalsConfig;
import com.revrobotics.spark.config.SoftLimitConfig;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;

import edu.wpi.first.math.geometry.Translation3d;
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
    public static final double kSpeedIn = 0.6;
    public static final double kSpeedOut = 0.6;
    public static final int kDIOSwitch = 0;
  }



  public static final class HandTiltConstants
  {
    public static final class Tilt {
      public static final int kIDHandTiltMotor = 26;
      public static final double kSpeedUp = 0.5;
      public static final double kSpeedDown = 0.4;
      public static final int kDIOtiltdownswitch = 1;

      public static final double kLoopRange[] = {-.8, .8};
      public static final double kIzone = 1.0; // DERGREES
      public static final double kPIDF[] = {400, 0, 0, 0};
      public static final double kPosConversion = 1.0;//180/32.7857
      public static final double kVelConversion = 1.0; // (180/30)/60
      public static final int kCPR = 28;
      public static final double ktiltLimit = 160.0;
      public static final double kmaxVel = 1000.0;              // [Max Inch/Sec] -> Affected by VelConversionFactor
      public static final double kmaxAcc = 1000.0;              // [Max Inch/Sec/Sec]
      public static final double kallowedError = 1.0;        // [Inches] -> Affected by PosConversionFactor
    
      // TILT MOTOR CONFIGURATION SUB-COMPONENTS (FOR CLEANER CODE)
      public static final ClosedLoopConfig kTiltLoopConfig = new ClosedLoopConfig()
                .feedbackSensor(FeedbackSensor.kAlternateOrExternalEncoder)
                .outputRange(kLoopRange[0], kLoopRange[1])
                .iZone(kIzone)
                .positionWrappingEnabled(false)
                .pidf(kPIDF[0], kPIDF[1], kPIDF[2], kPIDF[3]);
      
      public static final AlternateEncoderConfig kTiltEncoderConfig = new AlternateEncoderConfig() // MAY HAVE TO USE setSparkMaxDataPortConfig() method to configure the data port for alternate sensor mode.
                .positionConversionFactor(kPosConversion) // 
                .velocityConversionFactor(kVelConversion) // Default output is RPM.
                .inverted(true) // Positive motor direction should equal positive encoder movement.
                .countsPerRevolution(kCPR) // Encoder counts per revolution using Through Bore Encoder
                .measurementPeriod(10) // period in ms of the position measurement used for calculating the velocity. (Change in Position in Period)/(Period) = Velocity
                .averageDepth(32); // Default = 64 ~ Number of samples averaged for velocity reading, quadratures can be 1 to 64. 64 measurements average into one velocity measurement.

      public static final SoftLimitConfig kTiltSoftLimitConfig = new SoftLimitConfig()
                .forwardSoftLimit(ktiltLimit)
                .forwardSoftLimitEnabled(false)
                .reverseSoftLimit(0)
                .reverseSoftLimitEnabled(false);
      
      public static final MAXMotionConfig kMotorSmartMotion_Tilt = new MAXMotionConfig()
                .maxVelocity(kmaxVel)
                .maxAcceleration(kmaxAcc)
                .allowedClosedLoopError(kallowedError);
                
    }

    public static final class Grab {
      // GRABBER CONFIG
      public static final int kIDGrabbyThingy = 24;
      public static final int kIDGrabbyThingy2 = 25;

      public static final double kSpeedUp = 0.6; // OPEN
      public static final double kSpeedDown = 0.8; // CLOSE
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
     * [0][] - Position 0 - STARTING POSITION (Base is 2.7" forward of sensor, everything else is at zero sensor mark)
     * [1][] - Position 1 - Coral Pickup, Human Player
     * [2][] - Position 2 - Reef Base
     * [3][] - Position 3 - Reef Low
     * [4][] - Position 4 - Reef Mid
     * [5][] - Position 5 - Reef High
     * [6][] - Position 6 - Reef Algea Mid
     * [7][] - Position 7 - Reef Algea High
     * [8][] - Position 8 - 
     * [9][] - Position 9 - 
     * [][] - Position  - 
     * [][] - Position  - 
     * [][] - Position  - 
     */
      public static final double setpoint[][] = 
      {
     // {BOTTOM INCH, TOP INCH, EXTENSION INCH, HAND TILT DEGREES}
        {1.50,    0.0,   0.0,   0.1},     // POSITION 0 (STARTING POSITION)
        {0.00,    0.02,   0.00,   1.0},     // POSITION 1 (HUMAN PLAYER CORAL PICKUP)
        {0.00,    1.8,   0.00,   27},      // POSITION 2 (REEF BASE)
        {0.00,    3.3,   0.00,   30.5},      // POSITION 3 (REEF LOW)
        {0.00,    4.8,   0.00,   30.5},      // POSITION 4 (REEF MID)
        {0.00,    8.2,   6.3,   0.0},      // POSITION 5 (REEF HIGH)
        {0.3,    2.0,   0.00,   13},         // POSITION 6 (ALGEA CARRY)
        {0.00,    2.7,   0.0,   2},       // POSITION 7 (ALGEA LOW)
        {0.00,    5.02,   0.00,   0.0},       // POSITION 8 (ALGEA HIGH)
        {0.00,    8.6,   12.0,   27},       // POSITION 9 (ALGEA BARGE)
      };

      /** PLAYOFF SETPOINTS **/
    //   public static final double setpoint[][] = 
    //   {
    //  // {BOTTOM INCH, TOP INCH, EXTENSION INCH, HAND TILT DEGREES}
    //     {2.00,    0.0,   0.0,   0.1},     // POSITION 0 (STARTING POSITION)
    //     {0.00,    1.5,   0.00,   2},     // POSITION 1 (SAFE/HOME POSITION)
    //     {0.00,    3.7,   0.0,   2},      // POSITION 2 (REEF ALGEA LOW)
    //     {0.00,    5.3,   0.00,   2},      // POSITION 3 (REEF ALGEA HIGH)
    //     {0.3,    0.7,   0.00,   13},      // POSITION 4 (ALGEA CARRY)
    //     {2.0,    .86,   2.9,   2},      // POSITION 5 (ALGEA FLOOR)

    //     {0.00,    2.4,   0.00,   0.1},     // POSITION 6 (ALGEA LOW)
    //     {0.00,    5.00,   0.00,   0.1},       // POSITION 7 (ALGEA HIGH)

    //     {0.00,    0.02,   0.00,   0.0},       // POSITION 8 (WALL HP)

    //     {0.00,    5.00,   0.00,   30.0},       // POSITION 9
    //     {0.00,    5.00,   0.00,   30.0},       // POSITION 10
    //     {0.00,    5.00,   0.00,   30.0},       // POSITION 11
    //     {0.00,    5.00,   0.00,   30.0},       // POSITION 12
    //   };
    }
    
    /* TILT ARM CONSTANTS */
    public static final class Tilt
    {
      // IDs
      public static final int kIDArmTiltMotor = 22; // CAN ID
      public static final int kDIOBaseHomeSwitch = 4;     // DIO Port Number Zeroed Home Position Switch 0.0"
      public static final int kDIOBaseStartSwitch = 6;    // DIO Port Number Starting Position Switch ~1.5"
      // Speeds
      public static final double kSpeedUp = 0.5;    // Manual Speed to Tilt Arm Mechanism Backwards
      public static final double kspeedDown = 0.95;  // Manual Speed to Tilt Arm Mechanism Forwards (ONLY FOR CLIMBING/RESET)
      // Closed Loop Control
      public static final double[] kLoopRange = {-0.4,0.7}; // Allowable %Output of Closed Loop Controller (i.e. can't go faster then ±60%)
      public static final double[] kPIDF = {0.4, 0, 0, 0};   // {P, I, D, FF} Closed Loop Constants (F = 1/Kv from motor spec sheet if using velocity control, otherwise SET TO ZERO)
      public static final double kIzone = 5;              // Integral Constant Zone
    }
    
    /* LENGTH ARM CONSTANTS */
    public static final class Length
    {
      // IDs
      public static final int kIDArmTopLength = 23; // CAN ID
      public static final int kDIOHomeSwitch = 3;     // RoboRIO DIO Port Number
      public static final int kDIOMidSwitch = 5;
      public static final int kDIOTopSwitch = 7;
      // Speeds
      public static final double kSpeedUp = 0.4;    // Manual Speed to Lift Upper Arm Mechanism
      public static final double kspeedDown = 0.2;  // Manual Speed to Lower Upper Arm Mechanism
    }
    
    /* EXTEND ARM CONSTANTS */
    public static final class Extend
    {
      // IDs
      public static final int kIDextend = 28;
      public static final int kDIOextendretractswitch = 2;
      // Speeds
      public static final double kSpeedUp = 0.8;    // Manual Speed to Extend Upper Arm Mechanism
      public static final double kspeedDown = 0.5;  // Manual Speed to Retract Upper Arm Mechanism
    }

    /* ARM MOTOR CONFIGURATIONS */
    public static final class MotorConfigs
    {
      // ENCODER CONFIG CONSTANTS
      public static final double kPosConversion_NEO = 0.5;    // (Default output is rotations) Inch = Rotation * PosConversion = Rot * (0.5 inch / 1 rot)
      public static final double kVelConversion_NEO = kPosConversion_NEO/60;    // (Default output is RPM) Inch/Sec = RPM * VelConversion = RPM * (0.5 inch / 1 rev) * (1 min/ 60 sec)
      public static final double kPosConversion_HD = 8.25/182;    // (Default output is rotations) Inch = Rotation * PosConversion = Rot * ~~~~
      public static final double kVelConversion_HD = kPosConversion_HD/60;    // (Default output is RPM) Inch/Sec = RPM * VelConversion = RPM * ~~~~
      public static final double kPosConversion_TEST = 0.0;    // (Default output is rotations) Inch = Rotation * PosConversion = Rot * ~~~~
      public static final double kVelConversion_TEST = kPosConversion_HD/60;    // (Default output is RPM) Inch/Sec = RPM * VelConversion = RPM * ~~~~
      public static final int kCPR_RevBore = 8192;                // Encoder counts per revolution (Rev Throughbore = 8192)
      public static final int kCPR_HD = 28;                // Encoder counts per revolution (HD Built-in Encoder AT MOTOR SHAFT = 28) // Doesnt include gear box
      // MOTOR SOFT LIMITS
      public static final double kbaseBackLimit = -1.0;  // Soft limit in inches as read from encoder
      public static final double kbaseForwardLimit = 8;  // Soft limit in inches as read from encoder
      public static final double ktopDownLimit = -1.0;  // Soft limit in inches as read from encoder
      public static final double ktopTopLimit = 8.5;  // Soft limit in inches as read from encoder
      public static final double kextendExtendLimit = 12.0; // Soft limit in inches as read from encoder
      public static final double kextendRetractLimit = 0.0; // Soft limit in inches as read from encoder
      // CLOSED LOOP CONSTANTS

      // DELETE THIS SECTION ONCE CONVERTED OVER TO BOT AND TOP //
      public static final double[] kLoopRange_NEO = {-0.4,0.7}; // Allowable %Output of Closed Loop Controller (i.e. can't go faster then ±60%)
      public static final double[] kPIDF_NEO = {0.4, 0, 0, 1/473};   // {P, I, D, FF} Closed Loop Constants (F = 1/Kv from motor spec sheet if using velocity control, otherwise SET TO ZERO)
      public static final double kIzone_NEO = 0.1;              // Integral Constant Zone
      
      public static final double[] kLoopRange_Bot = {-0.6,0.8}; // Allowable %Output of Closed Loop Controller (i.e. can't go faster then ±60%)
      public static final double[] kPIDF_Bot = {20, 0, 0, 0};   // {P, I, D, FF} Closed Loop Constants (F = 1/Kv from motor spec sheet if using velocity control, otherwise SET TO ZERO)
      public static final double kIzone_Bot = 0.1;              // Integral Constant Zone
      public static final double kmaxVel_Bot = 2.0 / kVelConversion_NEO;              // [Max Inch/Sec] -> Affected by VelConversionFactor
      public static final double kmaxAcc_Bot = (kmaxVel_Bot/60.0) / kVelConversion_NEO;              // [Max Inch/Sec/Sec]
      public static final double kallowedError_Bot = 0.1;        // [Inches] -> Affected by PosConversionFactor
      

      public static final double[] kLoopRange_Top = {-0.65,0.65}; // Allowable %Output of Closed Loop Controller (i.e. can't go faster then ±60%)
      public static final double[] kPIDF_Top = {10, 0, 0, 0.0};   // {P, I, D, FF} Closed Loop Constants (F = 1/Kv from motor spec sheet if using velocity control, otherwise SET TO ZERO)
      public static final double kIzone_Top = 0.1;              // Integral Constant Zone
      public static final double kmaxVel_Top = 1.75 / (kVelConversion_NEO);              // [Max Inch/Sec] -> Affected by VelConversionFactor
      public static final double kmaxAcc_Top = 0.5 / (kVelConversion_NEO/60.0);              // [Max Inch/Sec/Sec]
      public static final double kallowedError_Top = 0.1;        // [Inches] -> Affected by PosConversionFactor


      public static final double[] kLoopRange_Extend = {-0.8,0.9}; // Allowable %Output of Closed Loop Controller (i.e. can't go faster then ±60%)
      public static final double[] kPIDF_Extend = {205, 0, 0, 0.0};   // {P, I, D, FF} Closed Loop Constants (F = 1/Kv from motor spec sheet if using velocity control, otherwise SET TO ZERO)
      public static final double kIzone_Extend = 0.3;              // Integral Constant Zone
      public static final double kmaxVel_Extend = 3.0 / (kVelConversion_HD);              // [Max Inch/Sec] -> Affected by VelConversionFactor
      public static final double kmaxAcc_Extend = 2.5 / (kVelConversion_HD/60);              // [Max Inch/Sec/Sec]
      public static final double kallowedError_Extend = 0.15;        // [Inches] -> Affected by PosConversionFactor

      // ALTERNATE ENCODER SPARK MAX CONFIGURATIONS
      public static final AlternateEncoderConfig kAltEncoderConfig_NEO = new AlternateEncoderConfig() // MAY HAVE TO USE setSparkMaxDataPortConfig() method to configure the data port for alternate sensor mode.
          .positionConversionFactor(kPosConversion_NEO) // 
          .velocityConversionFactor(kVelConversion_NEO) // Default output is RPM.
          .inverted(true) // Positive motor direction should equal positive encoder movement.
          .countsPerRevolution(kCPR_RevBore) // Encoder counts per revolution using Through Bore Encoder
          .measurementPeriod(10) // period in ms of the position measurement used for calculating the velocity. (Change in Position in Period)/(Period) = Velocity
          .averageDepth(64); // Default = 64 ~ Number of samples averaged for velocity reading, quadratures can be 1 to 64. 64 measurements average into one velocity measurement.
          public static final AlternateEncoderConfig kAltEncoderConfig_Top = new AlternateEncoderConfig() // MAY HAVE TO USE setSparkMaxDataPortConfig() method to configure the data port for alternate sensor mode.
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
          .averageDepth(64); // Default = 64 ~ Number of samples averaged for velocity reading, quadratures can be 1 to 64. 64 measurements average into one velocity measurement.
      public static final AlternateEncoderConfig kAltEncoderConfig_Extend = new AlternateEncoderConfig() // MAY HAVE TO USE setSparkMaxDataPortConfig() method to configure the data port for alternate sensor mode.
          .positionConversionFactor(kPosConversion_HD) // 
          .velocityConversionFactor(kVelConversion_HD) // Default output is RPM.
          .inverted(true) // Positive motor direction should equal positive encoder movement.
          .countsPerRevolution(kCPR_HD) // Encoder counts per revolution using Through Bore Encoder
          .measurementPeriod(10) // period in ms of the position measurement used for calculating the velocity. (Change in Position in Period)/(Period) = Velocity
          .averageDepth(64); // Default = 64 ~ Number of samples averaged for velocity reading, quadratures can be 1 to 64. 64 measurements average into one velocity measurement.
      
      // CLOSED LOOP SPARK MAX CONFIGURATIONS
      public static final ClosedLoopConfig kMotorLoopConfig_NEO = new ClosedLoopConfig()
          .feedbackSensor(FeedbackSensor.kAlternateOrExternalEncoder)
          .outputRange(kLoopRange_NEO[0], kLoopRange_NEO[1])
          .iZone(kIzone_NEO)
          .positionWrappingEnabled(false)
          .pidf(kPIDF_NEO[0], kPIDF_NEO[1], kPIDF_NEO[2], kPIDF_NEO[3]);

      public static final ClosedLoopConfig kMotorLoopConfig_Bot = new ClosedLoopConfig()
         .feedbackSensor(FeedbackSensor.kAlternateOrExternalEncoder)
         .outputRange(kLoopRange_Bot[0], kLoopRange_Bot[1])
          .iZone(kIzone_Bot)
          .positionWrappingEnabled(false)
          .pidf(kPIDF_Bot[0], kPIDF_Bot[1], kPIDF_Bot[2], kPIDF_Bot[3]);
      public static final MAXMotionConfig kMotorSmartMotion_Bot = new MAXMotionConfig()
            .maxVelocity(kmaxVel_Bot)
            .maxAcceleration(kmaxAcc_Bot)
            .allowedClosedLoopError(kallowedError_Bot);
      
      public static final ClosedLoopConfig kMotorLoopConfig_Top = new ClosedLoopConfig()
          .feedbackSensor(FeedbackSensor.kAlternateOrExternalEncoder)
          .outputRange(kLoopRange_Top[0], kLoopRange_Top[1])
          .iZone(kIzone_Top)
          .positionWrappingEnabled(false)
          .pidf(kPIDF_Top[0], kPIDF_Top[1], kPIDF_Top[2], kPIDF_Top[3]);
      public static final MAXMotionConfig kMotorSmartMotion_Top = new MAXMotionConfig()
          .maxVelocity(kmaxVel_Top)
          .maxAcceleration(kmaxAcc_Top)
          .allowedClosedLoopError(kallowedError_Top);
      
      public static final ClosedLoopConfig kMotorLoopConfig_Extend = new ClosedLoopConfig()
          .feedbackSensor(FeedbackSensor.kAlternateOrExternalEncoder)
          .outputRange(kLoopRange_Extend[0], kLoopRange_Extend[1])
          .iZone(kIzone_Extend)
          .positionWrappingEnabled(false)
          .pidf(kPIDF_Extend[0], kPIDF_Extend[1], kPIDF_Extend[2], kPIDF_Extend[3]);
      public static final MAXMotionConfig kMotorSmartMotion_Extend = new MAXMotionConfig()
          .maxVelocity(kmaxVel_Extend)
          .maxAcceleration(kmaxAcc_Extend)
          .allowedClosedLoopError(kallowedError_Extend);
      
      // SOFT LIMIT SPARK MAX CONFIGURATIONS
      public static final SoftLimitConfig kMotorSoftLimitConfig_Base = new SoftLimitConfig()
          .forwardSoftLimit(kbaseForwardLimit)
          .forwardSoftLimitEnabled(true)
          .reverseSoftLimit(kbaseBackLimit)
          .reverseSoftLimitEnabled(true);
      public static final SoftLimitConfig kMotorSoftLimitConfig_Top = new SoftLimitConfig()
          .forwardSoftLimit(ktopTopLimit)
          .forwardSoftLimitEnabled(true)
          .reverseSoftLimit(ktopDownLimit)
          .reverseSoftLimitEnabled(false);
      public static final SoftLimitConfig kMotorSoftLimitConfig_Extend = new SoftLimitConfig()
          .forwardSoftLimit(kextendExtendLimit)
          .forwardSoftLimitEnabled(true)
          .reverseSoftLimit(kextendRetractLimit)
          .reverseSoftLimitEnabled(true);
    }

    
  }





  
  public static final class DrivebaseConstants
  {
    // Hold time on motor brakes when disabled
    public static final double WHEEL_LOCK_TIME = 10; // seconds
    public static final double ROBOT_MASS = Units.lbsToKilograms(120); // 32lbs * kg per pound
    public static final Matter CHASSIS    = new Matter(new Translation3d(0, 0, Units.inchesToMeters(7)), ROBOT_MASS);
    public static final double LOOP_TIME  = 0.13; //s, 20ms + 110ms sprk max velocity lag
    public static final double MAX_SPEED  = 2.4; // Max speed of 2024 Chassis is ~5 m/s
    // Maximum speed of the robot in meters per second, used to limit acceleration.

    // DRIVEBASE CONSTANTS FOR AUTO ALIGNING TO THE CORAL HUMAN PLAYER STATION
    public static final double X_FEED_ALIGNMENT_P = 0.0; // PID Proportional Value
    public static final double Y_FEED_ALIGNMENT_P = 0.1; // PID Proportional Value
    public static final double ROT_FEED_ALIGNMENT_P = 0.0; // PID Proportional Value

    public static final double ROT_SETPOINT_FEED_ALIGNMENT = 0;  // Rotation = RY from LL
  	public static final double ROT_TOLERANCE_FEED_ALIGNMENT = 90; // ± Deg
  	public static final double X_SETPOINT_FEED_ALIGNMENT = 25.8;  // Vertical pose = TX from LL, It will essentially control the distance from the wall that we are
  	public static final double X_TOLERANCE_FEED_ALIGNMENT = 1.0; // ± Tol (DONT NEED X)
  	public static final double Y_SETPOINT_FEED_ALIGNMENT = 6.1;  // Horizontal pose = TZ from LL, Could need two different positons if you want robot to align from either side but lets stick with always aligning with the robot to the right of the april tag every time.
  	public static final double Y_TOLERANCE_FEED_ALIGNMENT = 1.0; // ± Tol

    // LIMELIGHT APRIL TAG AUTO ALIGN CONSTANTS
  	public static final double DONT_SEE_TAG_WAIT_TIME = 1;
  	public static final double POSE_VALIDATION_TIME = 0.3;
  }




  public static class OperatorConstants
  {

    // Joystick Deadband
    public static final double DEADBAND        = 0.12;
    public static final double LEFT_Y_DEADBAND = 0.12;
    public static final double RIGHT_X_DEADBAND = 0.12;
    public static final double TURN_CONSTANT    = 2.5;
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
        .absoluteEncoderPositionAlwaysOn(false)
        .absoluteEncoderVelocityAlwaysOn(false)
        .analogPositionAlwaysOn(false)
        .analogVelocityAlwaysOn(false)
        .analogVoltageAlwaysOn(false)
        .externalOrAltEncoderPositionAlwaysOn(true)
        .externalOrAltEncoderVelocityAlwaysOn(true)
        .externalOrAltEncoderPosition(10)
        .externalOrAltEncoderVelocity(10)
        .faultsAlwaysOn(false)
        .iAccumulationAlwaysOn(false)
        .primaryEncoderPositionAlwaysOn(false)
        .primaryEncoderVelocityAlwaysOn(false)
        
        .warningsAlwaysOn(true)
        .warningsPeriodMs(500); 
    }
    public static final class HandMotors
    {
      public static final SignalsConfig kMotorSignalConfig_Dumb = new SignalsConfig()
            .absoluteEncoderPositionAlwaysOn(false)
            .absoluteEncoderVelocityAlwaysOn(false)
            .analogPositionAlwaysOn(false)
            .analogVelocityAlwaysOn(false)
            .analogVoltageAlwaysOn(false)
            .faultsAlwaysOn(false)
            .iAccumulationAlwaysOn(false)
            .primaryEncoderPositionAlwaysOn(false)
            .primaryEncoderVelocityAlwaysOn(false)

            .warningsAlwaysOn(true)
            .warningsPeriodMs(500);

      public static final SignalsConfig kMotorSignalConfig_Tilt = new SignalsConfig()
      .absoluteEncoderPositionAlwaysOn(false)
      .absoluteEncoderVelocityAlwaysOn(false)
      .analogPositionAlwaysOn(false)
      .analogVelocityAlwaysOn(false)
      .analogVoltageAlwaysOn(false)
      .externalOrAltEncoderPositionAlwaysOn(true)
      .externalOrAltEncoderVelocityAlwaysOn(true)
      .externalOrAltEncoderPosition(10)
      .externalOrAltEncoderVelocity(10)
      .faultsAlwaysOn(false)
      .iAccumulationAlwaysOn(false)
      .primaryEncoderPositionAlwaysOn(false)
      .primaryEncoderVelocityAlwaysOn(false)

      .warningsAlwaysOn(true)
      .warningsPeriodMs(500);
    }
  }
}
