// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.events.EventTrigger;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.swervedrive.auto.Autos;
import frc.robot.subsystems.ArmExtend;
import frc.robot.subsystems.ArmLength;
import frc.robot.subsystems.ArmTilt;
import frc.robot.subsystems.FlyWheel;
import frc.robot.subsystems.HandTilt;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;
import java.io.File;
import swervelib.SwerveInputStream;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a "declarative" paradigm, very
 * little robot logic should actually be handled in the {@link Robot} periodic methods (other than the scheduler calls).
 * Instead, the structure of the robot (including subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer
{

  /* XBOX CONTROLLERS */
  final CommandXboxController driverXbox = new CommandXboxController(0);
  final CommandXboxController armXbox = new CommandXboxController(1);

  private final Trigger driveA = driverXbox.a();
  private final Trigger driveX = driverXbox.x();
  private final Trigger driveLB = driverXbox.leftBumper();
  private final Trigger driveRB = driverXbox.rightBumper();
  private final Trigger driveLT = driverXbox.leftTrigger(0.2);
  private final Trigger driveRT = driverXbox.rightTrigger(0.2);

  private final Trigger armA = armXbox.a();
  private final Trigger armB = armXbox.b();
  private final Trigger armX = armXbox.x();
  private final Trigger armY = armXbox.y();
  private final Trigger armLB = armXbox.leftBumper();
  private final Trigger armRB = armXbox.rightBumper();
  private final Trigger armLT = armXbox.leftTrigger();
  private final Trigger armRT = armXbox.rightTrigger();
  private final Trigger armSTRT = armXbox.start();
  private final Trigger armSLCT = armXbox.back();

  // The robot's subsystems and commands are defined here...
  private final SwerveSubsystem drivebase = new SwerveSubsystem(new File(Filesystem.getDeployDirectory(), "swerve"));
  private final ArmLength armlength = new ArmLength();
  private final ArmTilt armtilt = new ArmTilt();
  private final ArmExtend armExtend = new ArmExtend();
  private final HandTilt handtilt = new HandTilt();
  private final FlyWheel flywheel = new FlyWheel();

  //* TRIGGERS */
  // private final Trigger coraltrigger = new Trigger(flywheel::getSwitch);
  // private final Trigger handtiltTrigger = new Trigger(handtilt::getswitch);
  // private final Trigger armbasetrigger = new Trigger(armtilt::getbottomswitch);
  // private final Trigger armtoptrigger = new Trigger(armlength::gettopswitch);
  // private final Trigger armextendtrigger = new Trigger(armExtend::getSwitch);

  /** AUTOS */
  private static final String kDefaultAuto = "Default";
  private static final String kStraight = "Straight";
  private static final String kStraightL2 = "Straight Drop L2";
  private static final String kStraightL4 = "Straight Drop L4";
  private String autoScheduled;
  private final SendableChooser<String> mChooser = new SendableChooser<>();

  /**
   * Converts driver input into a field-relative ChassisSpeeds that is controlled by angular velocity.
   */
  SwerveInputStream driveAngularVelocity = SwerveInputStream.of(drivebase.getSwerveDrive(),
                                                                () -> driverXbox.getLeftY() * -1.0,
                                                                () -> driverXbox.getLeftX() * -1.0)
                                                            .withControllerRotationAxis(() -> driverXbox.getRightX() * -.8)
                                                            .deadband(OperatorConstants.DEADBAND)
                                                            .scaleTranslation(0.8)
                                                            .allianceRelativeControl(false);

  SwerveInputStream driveAngularVelocity_Slow = SwerveInputStream.of(drivebase.getSwerveDrive(),
                                                            () -> driverXbox.getLeftY() * -0.35,
                                                            () -> driverXbox.getLeftX() * -0.35)
                                                        .withControllerRotationAxis(() -> driverXbox.getRightX() * -.5)
                                                        .deadband(OperatorConstants.DEADBAND)
                                                        .scaleTranslation(0.8)
                                                        .allianceRelativeControl(false);

  /**
   * Clone's the angular velocity input stream and converts it to a fieldRelative input stream.
   */
  SwerveInputStream driveDirectAngle = driveAngularVelocity.copy().withControllerHeadingAxis(driverXbox::getRightX,
                                                                                             driverXbox::getRightY)
                                                           .headingWhile(true);



  // SwerveInputStream driveAngularVelocitySim = SwerveInputStream.of(drivebase.getSwerveDrive(),
  //                                                                  () -> -driverXbox.getLeftY(),
  //                                                                  () -> -driverXbox.getLeftX())
  //                                                              .withControllerRotationAxis(() -> driverXbox.getRawAxis(2))
  //                                                              .deadband(OperatorConstants.DEADBAND)
  //                                                              .scaleTranslation(0.8)
  //                                                              .allianceRelativeControl(true);
  // // Derive the heading axis with math!
  // SwerveInputStream driveDirectAngleSim     = driveAngularVelocitySim.copy()
  //                                                                    .withControllerHeadingAxis(() -> Math.sin(
  //                                                                                                   driverXbox.getRawAxis(
  //                                                                                                       2) * Math.PI) * (Math.PI * 2),
  //                                                                                               () -> Math.cos(
  //                                                                                                   driverXbox.getRawAxis(
  //                                                                                                       2) * Math.PI) *
  //                                                                                                     (Math.PI * 2))
  //                                                                    .headingWhile(true);

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer()
  {
    /** AUTOS */
    mChooser.setDefaultOption("Default", kDefaultAuto);
    mChooser.addOption("Straight", kStraight);
    mChooser.addOption("Straight Drop L2", kStraightL2);
    mChooser.addOption("Straight L4", kStraightL4);
    // mChooser.addOption("Straight Drop L4", kStraightL4);

    new EventTrigger("Open").onTrue(new InstantCommand(() -> armtilt.setSmartPosition(1), armtilt));
    new EventTrigger("Spit").onTrue(new StartEndCommand(flywheel::out, flywheel::stop, flywheel).withTimeout(6));
    new EventTrigger("L1").onTrue(new ParallelCommandGroup(
      new InstantCommand(() -> {handtilt.setSmartPosition(2);}, handtilt),
      new InstantCommand(() -> {armExtend.setSmartPosition(2);}, armExtend),
      new InstantCommand(() -> {armlength.setSmartPosition(2);}, armlength),
      new InstantCommand(() -> {armtilt.setSmartPosition(2);}, armtilt))
    );
    new EventTrigger("L4").onTrue(new ParallelCommandGroup(
      new InstantCommand(() -> {armExtend.setSmartPosition(5);}, armExtend),
      new InstantCommand(() -> {handtilt.setSmartPosition(5);}, handtilt),
      new InstantCommand(() -> {armlength.setSmartPosition(5);}, armlength),
      new InstantCommand(() -> {armtilt.setSmartPosition(5);}, armtilt))
    );
    new EventTrigger("Home").onTrue(new ParallelCommandGroup(
      new InstantCommand(() -> {handtilt.setSmartPosition(1);}, handtilt),
      new InstantCommand(() -> {armExtend.setSmartPosition(1);}, armExtend),
      new InstantCommand(() -> {armlength.setSmartPosition(1);}, armlength),
      new InstantCommand(() -> {armtilt.setSmartPosition(1);}, armtilt))
    );

    SmartDashboard.putData("Auto Select", mChooser);




    // Configure the trigger bindings
    configureBindingsDrive();
    configureBindingsOper();

    DriverStation.silenceJoystickConnectionWarning(true);
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary predicate, or via the
   * named factories in {@link edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for
   * {@link CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller PS4}
   * controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight joysticks}.
   */
  private void configureBindingsDrive()
  {
    // Command driveSetpointGen                      = drivebase.driveWithSetpointGeneratorFieldRelative(driveDirectAngle);
    // Command driveFieldOrientedDirectAngleSim      = drivebase.driveFieldOriented(driveDirectAngleSim);
    // Command driveFieldOrientedAnglularVelocitySim = drivebase.driveFieldOriented(driveAngularVelocitySim);
    // Command driveSetpointGenSim                   = drivebase.driveWithSetpointGeneratorFieldRelative(driveDirectAngleSim);

    // SET THE DRIVE TYPE
    Command driveFieldOrientedAnglularVelocity    = drivebase.driveFieldOriented(driveAngularVelocity);
    Command driveFieldOrientedAnglularVelocity_Slow    = drivebase.driveFieldOriented(driveAngularVelocity_Slow);
    drivebase.setDefaultCommand(driveFieldOrientedAnglularVelocity);
    // Command driveFieldOrientedDirectAngle         = drivebase.driveFieldOriented(driveDirectAngle);
    // drivebase.setDefaultCommand(driveFieldOrientedDirectAngle);

    /* COMPETITION/PRACTICE CONTROLS
      * A = ZERO GYRO
      * B = 
      * 
      * X = LOCK DRIVE BASE
      * Y = None
      * 
      * Left Bump = CLIMB UP / TILT COLUMN FORWARD
      * Right Bump = CLIMB DOWN / TILT COLUMN BACKWARD
      *
      * START = None
      * BACK = (FUTURE) OVERIDE TILT LIMIT SWITCH
    */
    driveA
      // .onTrue(Commands.runOnce(drivebase::zeroGyro, drivebase));
      .onTrue(new InstantCommand(drivebase::zeroGyro, drivebase));
    // driverXbox.b()
    //   .onTrue(drivebase.driveToPose(new Pose2d(new Translation2d(0, 0), Rotation2d.fromDegrees(180.0))).beforeStarting(drivebase::zeroGyro, drivebase));
    driveX
      // .whileTrue(Commands.runOnce(drivebase::lock, drivebase).repeatedly());
      .whileTrue(new InstantCommand(drivebase::lock, drivebase).repeatedly());
    // driverXbox.y()
    //   .whileTrue(drivebase.driveToDistanceCommand(2.0, 0.25)); // Drive Straight 2 Meters in 8 Seconds
    driveLB
      // .and(driverXbox.back().negate())
      // .onTrue(Commands.runOnce(armtilt::backwards, armtilt))  //MIGHT HAVE TO SWAP THIS ONE WITH RIGHT
      // .onFalse(Commands.runOnce(armtilt::halt, armtilt));
      .onTrue(new InstantCommand(armtilt::backwards, armtilt))  //MIGHT HAVE TO SWAP THIS ONE WITH RIGHT
      .onFalse(new InstantCommand(armtilt::halt, armtilt));
    driveRB
      // .and(driverXbox.back().negate())
      // .onTrue(Commands.runOnce(armtilt::forwards, armtilt))
      // .onFalse(Commands.runOnce(armtilt::halt, armtilt));
      .onTrue(new InstantCommand(armtilt::forwards, armtilt))
      .onFalse(new InstantCommand(armtilt::halt, armtilt));
    driveLT
      .or(driveRT)
      .whileTrue(driveFieldOrientedAnglularVelocity_Slow);
    // driverXbox.back()
    //   .and(driverXbox.leftBumper())
    //   .onTrue(Commands.none())
    // driverXbox.start().whileTrue(Commands.none()); 
  }

  public void configureBindingsOper()
  {
    /* COMPETITION PRACTICE
     * A = CAPTURE CORAL
     * B = RELEASE CORAL
     * 
     * X = CAPTURE ALGEA
     * Y = RELEASE ALGEA
     * 
     * Left Bump = HAND TILT UPWARDS
     * Right Bump = HAND TILT DOWNWARDS
     * Left Trigger = ARM LENGTH UPWARDS
     * Right Trigger = ARM LENGTH DOWNWARDS
     * BACK + TRIGGERS = ARM EXTEND
     *
     * BACK = CANCEL ALL COMMANDS (Prevent chaos if bad things happen)
     * START + BACK = Find Zeros and Go to Starting Positions
     * 
     * DPAD DOWN = LOW REEF CORAL
     * DPAD DOWN RIGHT = LOWER REEF ALGEA (NOT CONFIGURED)
     * DPAD RIGHT = MID REEF CORAL
     * DPAD UP RIGHT = HIGHER REEF ALGEA (NOT CONFIGURED)
     * DPAD UP = TOP REEF CORAL
     * DPAD UP LEFT = BARGE (NOT CONFIGURED)
     * DPAD LEFT = HOME/HUMAN PLAYER POSITION
     */

    // new InstantCommand(runnable, requirement) will initialize, execute, and end on the same iteration of the command scheduler.
    // Commands.runOnce(runnable, requirement) 
    armA
      .onTrue(Commands.runOnce(flywheel::in, flywheel))
      .onFalse(Commands.runOnce(flywheel::stop, flywheel));
      // .onTrue(new InstantCommand(flywheel::in, flywheel))
      // .onFalse(new InstantCommand(flywheel::stop, flywheel));
    
    armB
      .onTrue(Commands.runOnce(flywheel::out, flywheel))
      .onFalse(Commands.runOnce(flywheel::stop, flywheel));
      // .onTrue(new InstantCommand(flywheel::out, flywheel))
      // .onFalse(new InstantCommand(flywheel::stop, flywheel));
    
    armX
      .onTrue(Commands.runOnce(handtilt::close, handtilt))
      .onFalse(Commands.runOnce(handtilt::hold, handtilt));
      // .onTrue(new InstantCommand(handtilt::close, handtilt))
      // .onFalse(new InstantCommand(handtilt::hold, handtilt));
    
    armY
      .onTrue(Commands.runOnce(handtilt::open, handtilt))
      .onFalse(Commands.runOnce(handtilt::halt, handtilt));
      // .onTrue(new InstantCommand(handtilt::open, handtilt))
      // .onFalse(new InstantCommand(handtilt::halt, handtilt));
    
    armLT
      .and(armSLCT.negate())
      .onTrue(Commands.runOnce(armlength::Down, armlength))
      .onFalse(Commands.runOnce(armlength::Halt, armlength));
      // .onTrue(new InstantCommand(armlength::Down, armlength))
      // .onFalse(new InstantCommand(armlength::Halt, armlength));
    
    armRT
      .and(armSLCT.negate())
      .onTrue(Commands.runOnce(armlength::Up, armlength))
      .onFalse(Commands.runOnce(armlength::Halt, armlength));
      // .onTrue(new InstantCommand(armlength::Up, armlength))
      // .onFalse(new InstantCommand(armlength::Halt, armlength));
    
      armSLCT
      .and(armLT)
      .onTrue(Commands.runOnce(armExtend::out, armlength))
      .onFalse(Commands.runOnce(armExtend::stop, armlength));
      // .onTrue(new InstantCommand(armExtend::out, armlength))
      // .onFalse(new InstantCommand(armExtend::stop, armlength));
    
    armSLCT
      .and(armRT)
      .onTrue(Commands.runOnce(armExtend::in, armlength))
      .onFalse(Commands.runOnce(armExtend::stop, armlength));
      // .onTrue(new InstantCommand(armExtend::in, armlength))
      // .onFalse(new InstantCommand(armExtend::stop, armlength));
    
    armLB
      .onTrue(Commands.runOnce(handtilt::up, handtilt))
      .onFalse(Commands.runOnce(handtilt::stop, handtilt));
      // .onTrue(new InstantCommand(handtilt::up, handtilt))
      // .onFalse(new InstantCommand(handtilt::stop, handtilt));
    
    armRB
      .onTrue(Commands.runOnce(handtilt::down, handtilt))
      .onFalse(Commands.runOnce(handtilt::stop, handtilt));
      // .onTrue(new InstantCommand(handtilt::down, handtilt))
      // .onFalse(new InstantCommand(handtilt::stop, handtilt));

   // Zero out all the sensors and go to the starting position.
    armSTRT
      .and(armSLCT)
      .onTrue(new SequentialCommandGroup(
        new StartEndCommand(handtilt::up, handtilt::stop, handtilt).until(handtilt::getswitch).withTimeout(3),
        new StartEndCommand(armExtend::in, armExtend::stop, armExtend).until(armExtend::getSwitch).withTimeout(10),
        new StartEndCommand(armtilt::backwards, armtilt::halt, armtilt).until(armtilt::getHomeSwitch).withTimeout(10),
        new StartEndCommand(armtilt::forwards, armtilt::halt, armtilt).until(armtilt::getForwardSwitch).withTimeout(5),
        // new InstantCommand(() -> {armtilt.setSmartPosition(0);}, armtilt).withTimeout(20.0),
        new StartEndCommand(armlength::Down, armlength::Halt, armlength).until(armlength::getBottomSwitch).withTimeout(10)
      ));

    
     // * DPAD DOWN = LOW REEF CORAL
     // * DPAD DOWN RIGHT = LOWER REEF ALGEA

     // * DPAD RIGHT = MID REEF CORAL
     // * DPAD UP RIGHT = HIGHER REEF ALGEA
     // * DPAD UP = TOP REEF CORAL
     // * DPAD UP LEFT = BARGE
     // * DPAD LEFT = HOME/HUMAN PLAYER POSITION
     // * DPAD CENTER = MAKE THIS DEFAULT HOME??? (PROBS NOT)

    armSTRT.and(armXbox.povLeft()).onTrue( // DPAD LEFT + START - ALGEA SAFE TRAVEL
      new ParallelCommandGroup(
      new InstantCommand(() -> {handtilt.setSmartPosition(6);}, handtilt),
      new InstantCommand(() -> {armExtend.setSmartPosition(6);}, armExtend),
      new InstantCommand(() -> {armlength.setSmartPosition(6);}, armlength),
      new InstantCommand(() -> {armtilt.setSmartPosition(6);}, armtilt)
    ));
    armXbox.povLeft().onTrue( // DPAD LEFT - CORAL PICKUP
      new ParallelCommandGroup(
      new InstantCommand(() -> {handtilt.setSmartPosition(1);}, handtilt),
      new InstantCommand(() -> {armExtend.setSmartPosition(1);}, armExtend),
      new InstantCommand(() -> {armlength.setSmartPosition(1);}, armlength),
      new InstantCommand(() -> {armtilt.setSmartPosition(1);}, armtilt)
    ));
    
    armXbox.povDown().and(armSTRT).onTrue( // DPAD DOWN + START - ALGEA REEF LOW
      new ParallelCommandGroup(
      new InstantCommand(() -> {handtilt.setSmartPosition(7);}, handtilt),
      new InstantCommand(() -> {armExtend.setSmartPosition(7);}, armExtend),
      new InstantCommand(() -> {armlength.setSmartPosition(7);}, armlength),
      new InstantCommand(() -> {armtilt.setSmartPosition(7);}, armtilt)
    ));
    armXbox.povDown().onTrue( // DPAD DOWN DROP CORAL L2
      new ParallelCommandGroup(
      new InstantCommand(() -> {handtilt.setSmartPosition(3);}, handtilt),
      new InstantCommand(() -> {armExtend.setSmartPosition(3);}, armExtend),
      new InstantCommand(() -> {armlength.setSmartPosition(3);}, armlength),
      new InstantCommand(() -> {armtilt.setSmartPosition(3);}, armtilt)
    ));
    
    armSTRT.and(armXbox.povRight()).onTrue( // DPAD RIGHT + START - ALGEA REEF HIGH
      new ParallelCommandGroup(
      new InstantCommand(() -> {handtilt.setSmartPosition(8);}, handtilt),
      new InstantCommand(() -> {armExtend.setSmartPosition(8);}, armExtend),
      new InstantCommand(() -> {armlength.setSmartPosition(8);}, armlength),
      new InstantCommand(() -> {armtilt.setSmartPosition(8);}, armtilt)
    ));
    armXbox.povRight().onTrue( // DPAD RIGHT - CORAL L3
      new ParallelCommandGroup(
      new InstantCommand(() -> {handtilt.setSmartPosition(4);}, handtilt),
      new InstantCommand(() -> {armExtend.setSmartPosition(4);}, armExtend),
      new InstantCommand(() -> {armlength.setSmartPosition(4);}, armlength),
      new InstantCommand(() -> {armtilt.setSmartPosition(4);}, armtilt)
    ));

    armSTRT.and(armXbox.povUp()).onTrue( // DPAD UP + START - ALGEA BARGE
      new ParallelCommandGroup(
      new InstantCommand(() -> {handtilt.setSmartPosition(9);}, handtilt),
      new InstantCommand(() -> {armExtend.setSmartPosition(9);}, armExtend),
      new InstantCommand(() -> {armlength.setSmartPosition(9);}, armlength),
      new InstantCommand(() -> {armtilt.setSmartPosition(9);}, armtilt)
    ));
    armXbox.povUp().onTrue( // DPAD UP - CORAL L4
      new ParallelCommandGroup(
        new InstantCommand(() -> {armExtend.setSmartPosition(5);}, armExtend),
        new InstantCommand(() -> {handtilt.setSmartPosition(5);}, handtilt),
        new InstantCommand(() -> {armlength.setSmartPosition(5);}, armlength),
        new InstantCommand(() -> {armtilt.setSmartPosition(5);}, armtilt)
    ));

  }





  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand()
  {
    // An example command will be run in autonomous
    return drivebase.getAutonomousCommand(mChooser.getSelected());
  }




  public void setMotorBrake(boolean brake)
  {
    drivebase.setMotorBrake(brake);
  }

  public Command setPositions(int id) {
    return new ParallelCommandGroup(
      new InstantCommand(() -> {handtilt.setSmartPosition(1);}, handtilt),
      new InstantCommand(() -> {armExtend.setSmartPosition(1);}, armExtend),
      new InstantCommand(() -> {armlength.setSmartPosition(1);}, armlength),
      new InstantCommand(() -> {armtilt.setSmartPosition(1);}, armtilt)
    );
  }

}