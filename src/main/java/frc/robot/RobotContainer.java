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
  /** DECLARE THE CONTROLLERS 
   * Team 5623 convention is that the drivebase controls should always be
   * in the USB 0 slot of the driverstation and the secondary controls should
   * always be in the USB 1 slot of the driverstation.
   */
  final CommandXboxController driverXbox = new CommandXboxController(0);
  final CommandXboxController armXbox = new CommandXboxController(1);

  /** DECLARE THE BUTTONS OF THE CONTROLLERS
   * This technique appears to have resolved the issue seen prior to compentition
   * where the robot was randomly disabling itself because of a watchdog not fed
   * error that was a result of the robot.periodic loop overunning the 20ms loop time.
   */
  private final Trigger driveA = driverXbox.a();
  private final Trigger driveX = driverXbox.x();
  private final Trigger driveLB = driverXbox.leftBumper();
  private final Trigger driveRB = driverXbox.rightBumper();

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

  /** DECLARE THE SUBSYSTEMS OF THE ROBOT
   * Every subsystem in use should be declared here. For troubleshooting purposes
   * it is possible to disable one or more of these subsystem calls to isolate
   * any errors that arise or to just use select portions of the robot. Remember
   * to also comment out any place that it may be used later down the line.
   */
  private final SwerveSubsystem drivebase = new SwerveSubsystem(new File(Filesystem.getDeployDirectory(), "swerve"));
  private final ArmLength armlength = new ArmLength();
  private final ArmTilt armtilt = new ArmTilt();
  private final ArmExtend armExtend = new ArmExtend();
  private final HandTilt handtilt = new HandTilt();
  private final FlyWheel flywheel = new FlyWheel();

  /** DECLARE THE TRIGGERS ON THE ROBOT 
   * If using any sensors or condtions from any of the robot subsystems as
   * triggers, then declare those here after the subsystems.
   **/
  // private final Trigger coraltrigger = new Trigger(flywheel::getSwitch);
  // private final Trigger handtiltTrigger = new Trigger(handtilt::getswitch);
  // private final Trigger armbasetrigger = new Trigger(armtilt::getbottomswitch);
  // private final Trigger armtoptrigger = new Trigger(armlength::gettopswitch);
  // private final Trigger armextendtrigger = new Trigger(armExtend::getSwitch);

  /** DECLARE THE AUTO MODES AND DASHBBOARD SENDABLE CHOOSER
   * Start by creating a default string variable that will be autoselected if nothing else
   * is. Best practice is to have this option be a do nothing or a very minimal movement as
   * to not cause issues in a last minute selection.
   */
  private static final String kDefaultAuto = "Default";
  private static final String kStraight = "Straight";
  private static final String kStraightL2 = "Straight Drop L2";
  private static final String kStraightL4 = "Straight Drop L4";
  private String autoScheduled;
  private final SendableChooser<String> mChooser = new SendableChooser<>();

  /** DECLARE THE DEFAULT ROBOT DRIVING SCHEME
   * Converts driver input into a field-relative ChassisSpeeds that is controlled by angular velocity.
   */
  SwerveInputStream driveAngularVelocity = SwerveInputStream.of(drivebase.getSwerveDrive(),
                                                                () -> driverXbox.getLeftY() * -.9,
                                                                () -> driverXbox.getLeftX() * -.9)
                                                            .withControllerRotationAxis(driverXbox::getRightX)
                                                            .deadband(OperatorConstants.DEADBAND)
                                                            .scaleTranslation(0.8)
                                                            .allianceRelativeControl(false);

  
  /** CREATE A ROBOTCONTAINER
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer()
  {
    /** SEND THE AUTO OPTIONS TO THE DASHBOARD **/
    mChooser.setDefaultOption("Default", kDefaultAuto);
    mChooser.addOption("Straight", kStraight);
    // mChooser.addOption("Straight Drop L2", kStraightL2); DONT WORK WITH ALGEA CONTROLS
    // mChooser.addOption("Straight Drop L4", kStraightL4); DONT WORK WITH ALGEA CONTROLS

    /** CREATE PATHPLANNER EVENT TRIGGERS
     * Path planner requires that the event triggers used in paths and the named commands
     * that are used in autos, be declared prior to the sendable chooser and swerve subsystem
     * being assigned to minimize problems. The event triggers seemed to work at comp when
     * declared right here.
     */
    new EventTrigger("Open").onTrue(new InstantCommand(() -> armtilt.setSmartPosition(1), armtilt));
    new EventTrigger("Spit").onTrue(new StartEndCommand(flywheel::out, flywheel::stop, flywheel).withTimeout(6));
    new EventTrigger("L2").onTrue(new ParallelCommandGroup(
      new InstantCommand(() -> {handtilt.setSmartPosition(3);}, handtilt),
      new InstantCommand(() -> {armExtend.setSmartPosition(3);}, armExtend),
      new InstantCommand(() -> {armlength.setSmartPosition(3);}, armlength),
      new InstantCommand(() -> {armtilt.setSmartPosition(3);}, armtilt))
    );
    new EventTrigger("L4").onTrue(new ParallelCommandGroup(
      new InstantCommand(() -> {handtilt.setSmartPosition(5);}, handtilt),
      new InstantCommand(() -> {armExtend.setSmartPosition(5);}, armExtend),
      new InstantCommand(() -> {armlength.setSmartPosition(5);}, armlength),
      new InstantCommand(() -> {armtilt.setSmartPosition(5);}, armtilt))
    );

    /** PUT ALL THE AUTO DATA TO THE DASHBOARD
     * Do this after the path planner stuff but before the control bindings.
     */
    SmartDashboard.putData("Auto Select", mChooser);

    /** CONFIGURE THE BUTTON BINDINGS TO COMMANDS
     * The drivebase and secondary control bindings were seperated into seperate methods to clean
     * up the code and make it easier to find things.
     */
    configureBindingsDrive();
    configureBindingsOper();

    // If a joystick is not connected to the driver station for some reason this will silent all the warnings that pop up.
    DriverStation.silenceJoystickConnectionWarning(true);
  }

  /** DRIVER BINDINGS
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary predicate, or via the
   * named factories in {@link edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for
   * {@link CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller PS4}
   * controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight joysticks}.
   */
  private void configureBindingsDrive()
  {
    /** SET THE DEFAULT COMMAND FOR THE DRIVE BASE
     * This will set the default command for the subsystem when not interrupted by the command scheduler.
     * Drivers are used to driving with the driveFieldOriented using the Angular Velocity method. for the
     * Week 1 competition.
     */
    Command driveFieldOrientedAnglularVelocity    = drivebase.driveFieldOriented(driveAngularVelocity);
    drivebase.setDefaultCommand(driveFieldOrientedAnglularVelocity);

    driveA
      .onTrue(new InstantCommand(drivebase::zeroGyro, drivebase));
    driveX
      .whileTrue(new InstantCommand(drivebase::lock, drivebase).repeatedly());
    driveLB
      // .and(driverXbox.back().negate())
      .onTrue(new InstantCommand(armtilt::backwards, armtilt))  //MIGHT HAVE TO SWAP THIS ONE WITH RIGHT
      .onFalse(new InstantCommand(armtilt::halt, armtilt));
    driveRB
      // .and(driverXbox.back().negate())
      .onTrue(new InstantCommand(armtilt::forwards, armtilt))
      .onFalse(new InstantCommand(armtilt::halt, armtilt));
  }

  /** SECONDARY CONTROL BINDINGS
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary predicate, or via the
   * named factories in {@link edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for
   * {@link CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller PS4}
   * controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight joysticks}.
   */
  public void configureBindingsOper()
  {
    armA // Intake a coral
      .onTrue(Commands.runOnce(flywheel::in, flywheel))
      .onFalse(Commands.runOnce(flywheel::stop, flywheel));
    armB // Drop a coral
      .onTrue(Commands.runOnce(flywheel::out, flywheel))
      .onFalse(Commands.runOnce(flywheel::stop, flywheel));
    armX // Grab an algea
      .onTrue(Commands.runOnce(handtilt::close, handtilt))
      .onFalse(Commands.runOnce(handtilt::hold, handtilt));
    armY // Release an algea
      .onTrue(Commands.runOnce(handtilt::open, handtilt))
      .onFalse(Commands.runOnce(handtilt::halt, handtilt));
    armLT // Lower the arm length
      .and(armSLCT.negate())
      .onTrue(Commands.runOnce(armlength::Down, armlength))
      .onFalse(Commands.runOnce(armlength::Halt, armlength));
    armRT // Raise the arm length
      .and(armSLCT.negate())
      .onTrue(Commands.runOnce(armlength::Up, armlength))
      .onFalse(Commands.runOnce(armlength::Halt, armlength));
    armSLCT // Extend the arm
      .and(armLT)
      .onTrue(Commands.runOnce(armExtend::out, armlength))
      .onFalse(Commands.runOnce(armExtend::stop, armlength));
    armSLCT // Retract the arm
      .and(armRT)
      .onTrue(Commands.runOnce(armExtend::in, armlength))
      .onFalse(Commands.runOnce(armExtend::stop, armlength));
    armLB // Raise the hand finger thing
      .onTrue(Commands.runOnce(handtilt::up, handtilt))
      .onFalse(Commands.runOnce(handtilt::stop, handtilt));
    armRB // Lower the hand finger thing
      .onTrue(Commands.runOnce(handtilt::down, handtilt))
      .onFalse(Commands.runOnce(handtilt::stop, handtilt));

    armSTRT // Zero out all the sensors and go to the starting configuration position.
      .and(armSLCT)
      .onTrue(new SequentialCommandGroup(
        new StartEndCommand(handtilt::up, handtilt::stop, handtilt).until(handtilt::getswitch).withTimeout(3),
        new StartEndCommand(armExtend::in, armExtend::stop, armExtend).until(armExtend::getSwitch).withTimeout(10),
        new StartEndCommand(armtilt::backwards, armtilt::halt, armtilt).until(armtilt::getSwitch).withTimeout(20),
        new InstantCommand(() -> {armtilt.setSmartPosition(0);}, armtilt).withTimeout(20.0),
        new StartEndCommand(armlength::Down, armlength::Halt, armlength).until(armlength::getSwitch).withTimeout(10)
      ));

    armXbox.povLeft() // DPAD LEFT - ALGEA SAFE CARRY
      .onTrue( 
        new ParallelCommandGroup(
          new InstantCommand(() -> {handtilt.setSmartPosition(4);}, handtilt),
          new InstantCommand(() -> {armExtend.setSmartPosition(4);}, armExtend),
          new InstantCommand(() -> {armlength.setSmartPosition(4);}, armlength),
          new InstantCommand(() -> {armtilt.setSmartPosition(4);}, armtilt)
      ));
    // armSTRT
    //   .and(armXbox.povLeft())
    //   .onTrue( // DPAD LEFT + START - CORAL HOME / HP DANGEROUS PICKUP
    //     new ParallelCommandGroup(
    //       new InstantCommand(() -> {handtilt.setSmartPosition(1);}, handtilt),
    //       new InstantCommand(() -> {armExtend.setSmartPosition(1);}, armExtend),
    //       new InstantCommand(() -> {armlength.setSmartPosition(1);}, armlength),
    //       new InstantCommand(() -> {armtilt.setSmartPosition(1);}, armtilt)
    //   ));
    
    armXbox.povDown() // DPAD DOWN - ALGEA FLOOR PICKUP
      .onTrue( 
        new ParallelCommandGroup(
        new InstantCommand(() -> {handtilt.setSmartPosition(5);}, handtilt),  // WAS 3 FOR CORAL
        new InstantCommand(() -> {armExtend.setSmartPosition(5);}, armExtend),
        new InstantCommand(() -> {armlength.setSmartPosition(5);}, armlength),
        new InstantCommand(() -> {armtilt.setSmartPosition(5);}, armtilt)
      ));
    
    armXbox.povRight() // DPAD RIGHT - ALGEA REEF LOW PICKUP
      .onTrue(
        new ParallelCommandGroup(
          new InstantCommand(() -> {handtilt.setSmartPosition(2);}, handtilt),
          new InstantCommand(() -> {armExtend.setSmartPosition(2);}, armExtend),
          new InstantCommand(() -> {armlength.setSmartPosition(2);}, armlength),
          new InstantCommand(() -> {armtilt.setSmartPosition(2);}, armtilt)
      ));

    armXbox.povUp() // DPAD UP - ALGEA REEF HIGH PICKUP
      .onTrue(
        new ParallelCommandGroup(
          new InstantCommand(() -> {handtilt.setSmartPosition(3);}, handtilt),
          new InstantCommand(() -> {armExtend.setSmartPosition(3);}, armExtend),
          new InstantCommand(() -> {armlength.setSmartPosition(3);}, armlength),
          new InstantCommand(() -> {armtilt.setSmartPosition(3);}, armtilt)
      ));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   * 
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() { return drivebase.getAutonomousCommand(mChooser.getSelected()); }

  /**
   * Use this method to enable/disable the drivebase brake, which switches the
   * motor controller configuration between brake and coast modes.
   */ 
  public void setMotorBrake(boolean brake) { drivebase.setMotorBrake(brake); }

  /**
   * Use this command to set all the arm/hand positions to a smart controlled position
   * using the MaxMotion motion profiler. Pass through the id of the pre-configured location to use.
   */
  public Command setPositions(int id) {
    return new ParallelCommandGroup(
      new InstantCommand(() -> {handtilt.setSmartPosition(id);}, handtilt),
      new InstantCommand(() -> {armExtend.setSmartPosition(id);}, armExtend),
      new InstantCommand(() -> {armlength.setSmartPosition(id);}, armlength),
      new InstantCommand(() -> {armtilt.setSmartPosition(id);}, armtilt)
    );
  }
}
