// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.NamedCommands;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.Constants.OperatorConstants;
import frc.robot.Constants.ArmConst.kposition;
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

  // The robot's subsystems and commands are defined here...
  private final SwerveSubsystem drivebase = new SwerveSubsystem(new File(Filesystem.getDeployDirectory(), "swerve"));
  private final ArmLength armlength = new ArmLength();
  private final ArmTilt armtilt = new ArmTilt();
  private final ArmExtend armExtend = new ArmExtend();
  private final HandTilt handtilt = new HandTilt();
  private final FlyWheel flywheel = new FlyWheel();

  //* TRIGGERS */
  private final Trigger coraltrigger = new Trigger(flywheel::getSwitch);
  private final Trigger handtiltTrigger = new Trigger(handtilt::getswitch);
  // private final Trigger armbasetrigger = new Trigger(armtilt::getbottomswitch);
  // private final Trigger armtoptrigger = new Trigger(armlength::gettopswitch);
  // private final Trigger armextendtrigger = new Trigger(armExtend::getSwitch);


  /**
   * Converts driver input into a field-relative ChassisSpeeds that is controlled by angular velocity.
   */
  SwerveInputStream driveAngularVelocity = SwerveInputStream.of(drivebase.getSwerveDrive(),
                                                                () -> driverXbox.getLeftY() * .9,
                                                                () -> driverXbox.getLeftX() * .9)
                                                            .withControllerRotationAxis(driverXbox::getRightX)
                                                            .deadband(OperatorConstants.DEADBAND)
                                                            .scaleTranslation(0.8)
                                                            .allianceRelativeControl(true);

  /**
   * Clone's the angular velocity input stream and converts it to a fieldRelative input stream.
   */
  SwerveInputStream driveDirectAngle = driveAngularVelocity.copy().withControllerHeadingAxis(driverXbox::getRightX,
                                                                                             driverXbox::getRightY)
                                                           .headingWhile(true);


  SwerveInputStream driveAngularVelocitySim = SwerveInputStream.of(drivebase.getSwerveDrive(),
                                                                   () -> -driverXbox.getLeftY(),
                                                                   () -> -driverXbox.getLeftX())
                                                               .withControllerRotationAxis(() -> driverXbox.getRawAxis(2))
                                                               .deadband(OperatorConstants.DEADBAND)
                                                               .scaleTranslation(0.8)
                                                               .allianceRelativeControl(true);
  // Derive the heading axis with math!
  SwerveInputStream driveDirectAngleSim     = driveAngularVelocitySim.copy()
                                                                     .withControllerHeadingAxis(() -> Math.sin(
                                                                                                    driverXbox.getRawAxis(
                                                                                                        2) * Math.PI) * (Math.PI * 2),
                                                                                                () -> Math.cos(
                                                                                                    driverXbox.getRawAxis(
                                                                                                        2) * Math.PI) *
                                                                                                      (Math.PI * 2))
                                                                     .headingWhile(true);

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer()
  {
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
    // Command driveFieldOrientedDirectAngle         = drivebase.driveFieldOriented(driveDirectAngle);
    // Command driveSetpointGen                      = drivebase.driveWithSetpointGeneratorFieldRelative(driveDirectAngle);
    // Command driveFieldOrientedDirectAngleSim      = drivebase.driveFieldOriented(driveDirectAngleSim);
    // Command driveFieldOrientedAnglularVelocitySim = drivebase.driveFieldOriented(driveAngularVelocitySim);
    // Command driveSetpointGenSim                   = drivebase.driveWithSetpointGeneratorFieldRelative(driveDirectAngleSim);

    // SET THE DRIVE TYPE
    Command driveFieldOrientedAnglularVelocity    = drivebase.driveFieldOriented(driveAngularVelocity);
    drivebase.setDefaultCommand(driveFieldOrientedAnglularVelocity);

    /* COMPETITION/PRACTICE CONTROLS
      * A = ZERO GYRO
      * B = DO A 180!!!
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
    driverXbox.a()
      .onTrue(Commands.runOnce(drivebase::zeroGyro, drivebase));
    driverXbox.b()
      .onTrue(drivebase.driveToPose(new Pose2d(new Translation2d(0, 0), Rotation2d.fromDegrees(180.0))).beforeStarting(drivebase::zeroGyro, drivebase));
    driverXbox.x()
      .whileTrue(Commands.runOnce(drivebase::lock, drivebase).repeatedly());
    // driverXbox.y()
    //   .whileTrue(drivebase.driveToDistanceCommand(2.0, 0.25)); // Drive Straight 2 Meters in 8 Seconds
    driverXbox.leftBumper()
      // .and(driverXbox.back().negate())
      .onTrue(Commands.runOnce(armtilt::backwards, armtilt))  //MIGHT HAVE TO SWAP THIS ONE WITH RIGHT
      .onFalse(Commands.runOnce(armtilt::halt, armtilt));
    driverXbox.rightBumper()
      // .and(driverXbox.back().negate())
      .onTrue(Commands.runOnce(armtilt::forwards, armtilt))
      .onFalse(Commands.runOnce(armtilt::halt, armtilt));
    driverXbox.back()
      .and(driverXbox.leftBumper())
      .onTrue(Commands.none())
      .onFalse(Commands.none());
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
    armXbox.a()
      .onTrue(Commands.runOnce(flywheel::in, flywheel)
          .until(coraltrigger))
      .onFalse(Commands.runOnce(flywheel::stop, flywheel));
    armXbox.b()
      .onTrue(Commands.runOnce(flywheel::out, flywheel))
      .onFalse(Commands.runOnce(flywheel::stop, flywheel));
    armXbox.x()
      .onTrue(Commands.runOnce(handtilt::close, handtilt))
      .onFalse(Commands.runOnce(handtilt::hold, handtilt));
    armXbox.y()
      .onTrue(Commands.runOnce(handtilt::open, handtilt))
      .onFalse(Commands.runOnce(handtilt::halt, handtilt));
    armXbox.leftTrigger()
      .and(armXbox.back().negate())
      .onTrue(Commands.runOnce(armlength::Up, armlength))
      .onFalse(Commands.runOnce(armlength::Halt, armlength));
    armXbox.rightTrigger()
      .and(armXbox.back().negate())
      .onTrue(Commands.runOnce(armlength::Down, armlength))
      .onFalse(Commands.runOnce(armlength::Halt, armlength));
      armXbox.back()
      .and(armXbox.leftTrigger())
      .onTrue(Commands.runOnce(armExtend::out, armlength))
      .onFalse(Commands.runOnce(armExtend::stop, armlength));
    armXbox.back()
      .and(armXbox.rightTrigger())
      .onTrue(Commands.runOnce(armExtend::in, armlength))
      .onFalse(Commands.runOnce(armExtend::stop, armlength));
    armXbox.leftBumper()
      .onTrue(Commands.runOnce(handtilt::up, handtilt))
      .onFalse(Commands.runOnce(handtilt::stop, handtilt));
    armXbox.rightBumper()
      .onTrue(Commands.runOnce(handtilt::down, handtilt))
      .onFalse(Commands.runOnce(handtilt::stop, handtilt));
    // armXbox.back()
    //   .onTrue(() -> CommandScheduler.getInstance().cancelAll())
    //   .onFalse(Commands.none());
    armXbox.start()
      .and(armXbox.back())
      .onTrue(new SequentialCommandGroup(
        new StartEndCommand(handtilt::up, handtilt::stop, handtilt).until(handtilt::getswitch).withTimeout(5),
        new StartEndCommand(armExtend::in, armExtend::stop, armExtend).until(armExtend::getSwitch).withTimeout(10),
        new StartEndCommand(armtilt::backwards, armtilt::halt, armtilt).until(armtilt::getSwitch).withTimeout(20),
        new InstantCommand(() -> {armtilt.setSmartPosition(0);}, armtilt).withTimeout(10.0).withTimeout(20),
        new StartEndCommand(armlength::Down, armlength::Halt, armlength).until(armlength::getSwitch).withTimeout(10)
      ))
      .onFalse(Commands.none());

    
     // * DPAD DOWN = LOW REEF CORAL
     // * DPAD DOWN RIGHT = LOWER REEF ALGEA
     // * DPAD RIGHT = MID REEF CORAL
     // * DPAD UP RIGHT = HIGHER REEF ALGEA
     // * DPAD UP = TOP REEF CORAL
     // * DPAD UP LEFT = BARGE
     // * DPAD LEFT = HOME/HUMAN PLAYER POSITION
     // * DPAD CENTER = MAKE THIS DEFAULT HOME??? (PROBS NOT)
    armXbox.povLeft().onTrue(
      new ParallelCommandGroup(
        new InstantCommand(() -> {handtilt.setSmartPosition(1);}, handtilt),
        new InstantCommand(() -> {armExtend.setSmartPosition(1);}, armExtend),
        new InstantCommand(() -> {armlength.setSmartPosition(1);}, armlength),
        new InstantCommand(() -> {armtilt.setSmartPosition(1);}, armtilt)
      ));
    
    armXbox.povDown().onTrue(
      new ParallelCommandGroup(
        new InstantCommand(() -> {handtilt.setSmartPosition(3);}, handtilt),
        new InstantCommand(() -> {armExtend.setSmartPosition(3);}, armExtend),
        new InstantCommand(() -> {armlength.setSmartPosition(3);}, armlength),
        new InstantCommand(() -> {armtilt.setSmartPosition(3);}, armtilt)
      ));

    armXbox.povDownRight().onTrue( // 6
      new ParallelCommandGroup(
        new InstantCommand(() -> {handtilt.setSmartPosition(6);}, handtilt),
        new InstantCommand(() -> {armExtend.setSmartPosition(6);}, armExtend),
        new InstantCommand(() -> {armlength.setSmartPosition(6);}, armlength),
        new InstantCommand(() -> {armtilt.setSmartPosition(6);}, armtilt)
      ));
    
    armXbox.povRight().onTrue(
      new ParallelCommandGroup(
        new InstantCommand(() -> {handtilt.setSmartPosition(4);}, handtilt),
        new InstantCommand(() -> {armExtend.setSmartPosition(4);}, armExtend),
        new InstantCommand(() -> {armlength.setSmartPosition(4);}, armlength),
        new InstantCommand(() -> {armtilt.setSmartPosition(4);}, armtilt)
      ));

    armXbox.povUpRight().onTrue( // 7
      new ParallelCommandGroup(
        new InstantCommand(() -> {handtilt.setSmartPosition(1);}, handtilt),
        new InstantCommand(() -> {armExtend.setSmartPosition(1);}, armExtend),
        new InstantCommand(() -> {armlength.setSmartPosition(1);}, armlength),
        new InstantCommand(() -> {armtilt.setSmartPosition(1);}, armtilt)
      ));

    armXbox.povUp().onTrue(
      new ParallelCommandGroup(
        new InstantCommand(() -> {handtilt.setSmartPosition(5);}, handtilt),
        new InstantCommand(() -> {armExtend.setSmartPosition(5);}, armExtend),
        new InstantCommand(() -> {armlength.setSmartPosition(5);}, armlength),
        new InstantCommand(() -> {armtilt.setSmartPosition(5);}, armtilt)
      ));

    armXbox.povUpLeft().onTrue( // 8
      new ParallelCommandGroup(
        new InstantCommand(() -> {handtilt.setSmartPosition(1);}, handtilt),
        new InstantCommand(() -> {armExtend.setSmartPosition(1);}, armExtend),
        new InstantCommand(() -> {armlength.setSmartPosition(1);}, armlength),
        new InstantCommand(() -> {armtilt.setSmartPosition(1);}, armtilt)
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
    return drivebase.getAutonomousCommand("New Auto");
  }

  public void setMotorBrake(boolean brake)
  {
    drivebase.setMotorBrake(brake);
  }

}
