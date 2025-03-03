package frc.robot;

import com.revrobotics.spark.config.ClosedLoopConfig;
import com.revrobotics.spark.config.EncoderConfig;
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
      public static final double kSpeedDown = 0.6; // CLOSE
    }
  }


  public static final class ArmConst
  {
    public static final class kposition
    {
    //   /** QUALIFICATION SETPOINTS (CORAL) **/
    //   public static final double setpoint[][] = 
    //   {
    //   // {BOTTOM INCH, TOP INCH, EXTENSION INCH, HAND TILT DEGREES}
    //     {2.00,    0.0,   0.0,   0.1},     // POSITION [0][] (STARTING POSITION)
    //     {0.00,    2.0,   0.00,   18},     // POSITION [1][] (HUMAN PLAYER POINTY PICKUP)
    //     {0.00,    3.0,   8.25,   10},     // POSITION [2][] (REEF BASE)
    //     {0.00,    3.3,   0.00,   30.5},   // POSITION [3][] (REEF LOW)
    //     {0.00,    4.8,   0.00,   30.5},   // POSITION [4][] (REEF MID)
    //     {0.00,    8.2,   8.75,   0.0},    // POSITION [5][] (REEF HIGH)
    //     {0.00,    2.4,   0.00,   0.1},    // POSITION [6][] (ALGEA LOW)
    //     {0.00,    5.00,   0.00,   0.1},   // POSITION [7][] (ALGEA HIGH)
    //     {0.00,    0.02,   0.00,   0.0}    // POSITION [8][] (HUMAN PLAYER WALL PICKUP)
    //   };

      /** PLAYOFF SETPOINTS (ALGEA) **/
      public static final double setpoint[][] = 
      {
      // {BOTTOM INCH, TOP INCH, EXTENSION INCH, HAND TILT DEGREES}
        {2.00,    0.0,   0.0,   0.1},     // POSITION 0 (STARTING POSITION)
        {0.00,    1.5,   0.00,   2},      // POSITION 1 (SAFE/HOME POSITION)
        {0.00,    3.7,   0.0,   2},       // POSITION 2 (REEF ALGEA LOW)
        {0.00,    5.3,   0.00,   2},      // POSITION 3 (REEF ALGEA HIGH)
        {0.3,    0.7,   0.00,   13},      // POSITION 4 (ALGEA CARRY)
        {2.0,    .86,   2.9,   2},        // POSITION 5 (ALGEA FLOOR)
        {0.00,    2.4,   0.00,   0.1},    // POSITION 6 (ALGEA LOW)
        {0.00,    5.00,   0.00,   0.1}    // POSITION 7 (ALGEA HIGH)
      };
    }
    
    /* TILT ARM CONSTANTS */
    public static final class Tilt
    {
      // IDs
      public static final int kIDArmTiltMotor = 22;         // CAN ID
      public static final int kDIOBaseExtendSwitch = 4;     // RoboRIO DIO Port Number
      // Speeds
      public static final double kSpeedUp = 0.5;            // Manual Speed to Tilt Arm Mechanism Backwards
      public static final double kspeedDown = 0.5;          // Manual Speed to Tilt Arm Mechanism Forwards (ONLY FOR CLIMBING/RESET)
      // Closed Loop Control
      public static final double[] kLoopRange = {-0.4,0.7}; // Allowable %Output of Closed Loop Controller (i.e. can't go faster then ±60%)
      public static final double[] kPIDF = {0.4, 0, 0, 0};  // {P, I, D, FF} Closed Loop Constants (F = 1/Kv from motor spec sheet if using velocity control, otherwise SET TO ZERO)
      public static final double kIzone = 5;                // Integral Constant Zone
    }
    
    /* LENGTH ARM CONSTANTS */
    public static final class Length
    {
      // IDs
      public static final int kIDArmTopLength = 23;     // CAN ID
      public static final int kDIOTopRetractSwitch = 3; // RoboRIO DIO Port Number
      // Speeds
      public static final double kSpeedUp = 0.5;        // Manual Speed to Lift Upper Arm Mechanism
      public static final double kspeedDown = 0.2;      // Manual Speed to Lower Upper Arm Mechanism
    }
    
    /* EXTEND ARM CONSTANTS */
    public static final class Extend
    {
      // IDs
      public static final int kIDextend = 28;              // CAN ID
      public static final int kDIOextendretractswitch = 2; // RoboRIO DIO Port Number
      // Speeds
      public static final double kSpeedUp = 0.5;           // Manual Speed to Extend Upper Arm Mechanism
      public static final double kspeedDown = 0.5;         // Manual Speed to Retract Upper Arm Mechanism
    }

    /* ARM MOTOR CONFIGURATIONS */
    public static final class MotorConfigs
    {
      // ENCODER CONFIG CONSTANTS
      public static final double kPosConversion_NEO = 0.5;                 // (Default output is rotations) Inch = Rotation * PosConversion = Rot * (0.5 inch / 1 rot)
      public static final double kVelConversion_NEO = 0.5/60;              // (Default output is RPM) Inch/Sec = RPM * VelConversion = RPM * (0.5 inch / 1 rev) * (1 min/ 60 sec)
      public static final double kPosConversion_HD = 8.25/182;             // (Default output is rotations) Inch = Rotation * PosConversion = Rot * ~~~~
      public static final double kVelConversion_HD = kPosConversion_HD/60; // (Default output is RPM) Inch/Sec = RPM * VelConversion = RPM * ~~~~
      public static final int kCPR_RevBore = 8192;                         // Encoder counts per revolution (Rev Throughbore = 8192)
      public static final int kCPR_HD = 28;                                // Encoder counts per revolution (HD Built-in Encoder AT MOTOR SHAFT = 28) // Doesnt include gear box
      // MOTOR SOFT LIMITS
      public static final double kbaseExtendLimit = 4.0;   // Soft limit in inches as read from encoder
      public static final double ktopRetractLimit = 4.0;   // Soft limit in inches as read from encoder
      public static final double kextendExtendLimit = 5.0; // Soft limit in inches as read from encoder
      // CLOSED LOOP CONSTANTS

      // DELETE THIS SECTION ONCE CONVERTED OVER TO BOT AND TOP //
      public static final double[] kLoopRange_NEO = {-0.4,0.7};    // Allowable %Output of Closed Loop Controller (i.e. can't go faster then ±60%)
      public static final double[] kPIDF_NEO = {0.4, 0, 0, 1/473}; // {P, I, D, FF} Closed Loop Constants (F = 1/Kv from motor spec sheet if using velocity control, otherwise SET TO ZERO)
      public static final double kIzone_NEO = 0.1;                 // Integral Constant Zone
      
      public static final double[] kLoopRange_Bot = {-0.6,0.8};  // Allowable %Output of Closed Loop Controller (i.e. can't go faster then ±60%)
      public static final double[] kPIDF_Bot = {20, 0, 0, 0};    // {P, I, D, FF} Closed Loop Constants (F = 1/Kv from motor spec sheet if using velocity control, otherwise SET TO ZERO)
      public static final double kIzone_Bot = 0.1;               // Integral Constant Zone
      public static final double kmaxVel_Bot = 2.0 / kVelConversion_NEO;                // [Max Inch/Sec] -> Affected by VelConversionFactor
      public static final double kmaxAcc_Bot = (kmaxVel_Bot/60.0) / kVelConversion_NEO; // [Max Inch/Sec/Sec]
      public static final double kallowedError_Bot = 0.1;        // [Inches] -> Affected by PosConversionFactor
      
      public static final double[] kLoopRange_Top = {-0.8,0.8};            // Allowable %Output of Closed Loop Controller (i.e. can't go faster then ±60%)
      public static final double[] kPIDF_Top = {15, 0, 0, 0.0};            // {P, I, D, FF} Closed Loop Constants (F = 1/Kv from motor spec sheet if using velocity control, otherwise SET TO ZERO)
      public static final double kIzone_Top = 0.1;                         // Integral Constant Zone
      public static final double kmaxVel_Top = 50.0 / kVelConversion_NEO;  // [Max Inch/Sec] -> Affected by VelConversionFactor
      public static final double kmaxAcc_Top = (4.0) / kVelConversion_NEO; // [Max Inch/Sec/Sec]
      public static final double kallowedError_Top = 0.1;                  // [Inches] -> Affected by PosConversionFactor

      public static final double[] kLoopRange_Extend = {-0.8,0.8};            // Allowable %Output of Closed Loop Controller (i.e. can't go faster then ±60%)
      public static final double[] kPIDF_Extend = {30, 0, 0, 0.0};            // {P, I, D, FF} Closed Loop Constants (F = 1/Kv from motor spec sheet if using velocity control, otherwise SET TO ZERO)
      public static final double kIzone_Extend = 0.1;                         // Integral Constant Zone
      public static final double kmaxVel_Extend = 40.0 / kVelConversion_NEO;  // [Max Inch/Sec] -> Affected by VelConversionFactor
      public static final double kmaxAcc_Extend = (5.0) / kVelConversion_NEO; // [Max Inch/Sec/Sec]
      public static final double kallowedError_Extend = 0.1;                  // [Inches] -> Affected by PosConversionFactor

      // ALTERNATE ENCODER SPARK MAX CONFIGURATIONS
      public static final AlternateEncoderConfig kAltEncoderConfig_NEO = new AlternateEncoderConfig() // MAY HAVE TO USE setSparkMaxDataPortConfig() method to configure the data port for alternate sensor mode.
          .positionConversionFactor(kPosConversion_NEO) // Defaul output is revolutions
          .velocityConversionFactor(kVelConversion_NEO) // Default output is RPM.
          .inverted(true) // Positive motor direction should equal positive encoder movement.
          .countsPerRevolution(kCPR_RevBore) // Encoder counts per revolution using Through Bore Encoder
          .measurementPeriod(10) // period in ms of the position measurement used for calculating the velocity. (Change in Position in Period)/(Period) = Velocity
          .averageDepth(64); // Default = 64 ~ Number of samples averaged for velocity reading, quadratures can be 1 to 64. 64 measurements average into one velocity measurement.
          public static final AlternateEncoderConfig kAltEncoderConfig_Top = new AlternateEncoderConfig() // MAY HAVE TO USE setSparkMaxDataPortConfig() method to configure the data port for alternate sensor mode.
          .positionConversionFactor(kPosConversion_NEO) // Defaul output is revolutions
          .velocityConversionFactor(kVelConversion_NEO) // Default output is RPM.
          .inverted(false) // Positive motor direction should equal positive encoder movement.
          .countsPerRevolution(kCPR_RevBore) // Encoder counts per revolution using Through Bore Encoder
          .measurementPeriod(10) // period in ms of the position measurement used for calculating the velocity. (Change in Position in Period)/(Period) = Velocity
          .averageDepth(64); // Default = 64 ~ Number of samples averaged for velocity reading, quadratures can be 1 to 64. 64 measurements average into one velocity measurement.
      public static final AlternateEncoderConfig kAltEncoderConfig_HD = new AlternateEncoderConfig() // MAY HAVE TO USE setSparkMaxDataPortConfig() method to configure the data port for alternate sensor mode.
          .positionConversionFactor(kPosConversion_HD) // Defaul output is revolutions
          .velocityConversionFactor(kVelConversion_HD) // Default output is RPM.
          .inverted(false) // Positive motor direction should equal positive encoder movement.
          .countsPerRevolution(kCPR_HD) // Encoder counts per revolution using Through Bore Encoder
          .measurementPeriod(10) // period in ms of the position measurement used for calculating the velocity. (Change in Position in Period)/(Period) = Velocity
          .averageDepth(64); // Default = 64 ~ Number of samples averaged for velocity reading, quadratures can be 1 to 64. 64 measurements average into one velocity measurement.
      public static final AlternateEncoderConfig kAltEncoderConfig_Extend = new AlternateEncoderConfig() // MAY HAVE TO USE setSparkMaxDataPortConfig() method to configure the data port for alternate sensor mode.
          .positionConversionFactor(kPosConversion_HD) // Defaul output is revolutions
          .velocityConversionFactor(kVelConversion_HD) // Default output is RPM.
          .inverted(true) // Positive motor direction should equal positive encoder movement.
          .countsPerRevolution(kCPR_HD) // Encoder counts per revolution using Through Bore Encoder
          .measurementPeriod(10) // period in ms of the position measurement used for calculating the velocity. (Change in Position in Period)/(Period) = Velocity
          .averageDepth(64); // Default = 64 ~ Number of samples averaged for velocity reading, quadratures can be 1 to 64. 64 measurements average into one velocity measurement.
      
      /** CLOSED LOOP SPARK MAX CONFIGURATIONS **/
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
      public static final ClosedLoopConfig kMotorLoopConfig_Top = new ClosedLoopConfig()
          .feedbackSensor(FeedbackSensor.kAlternateOrExternalEncoder)
          .outputRange(kLoopRange_Top[0], kLoopRange_Top[1])
          .iZone(kIzone_Top)
          .positionWrappingEnabled(false)
          .pidf(kPIDF_Top[0], kPIDF_Top[1], kPIDF_Top[2], kPIDF_Top[3]);
      public static final ClosedLoopConfig kMotorLoopConfig_Extend = new ClosedLoopConfig()
          .feedbackSensor(FeedbackSensor.kAlternateOrExternalEncoder)
          .outputRange(kLoopRange_Extend[0], kLoopRange_Extend[1])
          .iZone(kIzone_Extend)
          .positionWrappingEnabled(false)
          .pidf(kPIDF_Extend[0], kPIDF_Extend[1], kPIDF_Extend[2], kPIDF_Extend[3]);

      /** SPARK SMART MOTOR MOTION CONFIGURATION **/
      public static final MAXMotionConfig kMotorSmartMotion_Bot = new MAXMotionConfig()
            .maxVelocity(kmaxVel_Bot)
            .maxAcceleration(kmaxAcc_Bot)
            .allowedClosedLoopError(kallowedError_Bot);
      public static final MAXMotionConfig kMotorSmartMotion_Top = new MAXMotionConfig()
          .maxVelocity(kmaxVel_Top)
          .maxAcceleration(kmaxAcc_Top)
          .allowedClosedLoopError(kallowedError_Top);
      public static final MAXMotionConfig kMotorSmartMotion_Extend = new MAXMotionConfig()
          .maxVelocity(kmaxVel_Extend)
          .maxAcceleration(kmaxAcc_Extend)
          .allowedClosedLoopError(kallowedError_Extend);
      
      /** SOFT LIMIT SPARK MAX CONFIGURATIONS **/
      public static final SoftLimitConfig kMotorSoftLimitConfig_Base = new SoftLimitConfig()
          .forwardSoftLimit(kbaseExtendLimit)
          .forwardSoftLimitEnabled(false)
          .reverseSoftLimit(0.0)
          .reverseSoftLimitEnabled(false);
      public static final SoftLimitConfig kMotorSoftLimitConfig_Top = new SoftLimitConfig()
          .forwardSoftLimit(0.0)
          .forwardSoftLimitEnabled(false)
          .reverseSoftLimit(ktopRetractLimit)
          .reverseSoftLimitEnabled(false);
      public static final SoftLimitConfig kMotorSoftLimitConfig_Extend = new SoftLimitConfig()
          .forwardSoftLimit(kextendExtendLimit)
          .forwardSoftLimitEnabled(false)
          .reverseSoftLimit(0.0)
          .reverseSoftLimitEnabled(false);
    }
  }

  public static final class DrivebaseConstants
  {
    public static final double WHEEL_LOCK_TIME = 10;  // [seconds] Hold time on motor brakes when disabled
    public static final double ROBOT_MASS = Units.lbsToKilograms(110); // [kg] is ~105 lbs with bumber and no battery
    public static final Matter CHASSIS    = new Matter(new Translation3d(0, 0, Units.inchesToMeters(7)), ROBOT_MASS);
    public static final double LOOP_TIME  = 0.13;    // [seconds] 20ms + 110ms sprk max velocity lag
    public static final double MAX_SPEED  = 2.1;     // [m/s] Max speed of 2024 Chassis is ~5 m/s. This is used to limit acceleration in swerve code.
  }

  public static class OperatorConstants
  {
    public static final double DEADBAND        = 0.12;  // Joystick deadband
    public static final double LEFT_Y_DEADBAND = 0.12;  // Left Stick translation deadband
    public static final double RIGHT_X_DEADBAND = 0.12; // Right stick rotation deadband
    public static final double TURN_CONSTANT    = 2.5;  // ???
  }

  /**
   * It is best to start making the SignalsConfig in the actual subsystem that it is used in
   * in order to avoid having the Sparks throw configuration id errors for not apparent reason.
   * All the configuration parameters are available when setting it up in the constants file,
   * but only the applicable configurations are available when setting it up in the subsystem.
  */
  public static final class CANSignals
  {
    public static final class ArmMotors
    {
      // USED FOR THE THREE ARM MOTORS BECAUSE THEY HAVE SMART CONTROLLING
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
      // USED FOR THE GRABBER MOTORS WHICH HAVE NO SPECIAL SMART CONTORLLING
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
      // USED FOR THE HAND TILT MOTOR BECAUSE IT HAS SPECIAL SMART CONTROLLING
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
