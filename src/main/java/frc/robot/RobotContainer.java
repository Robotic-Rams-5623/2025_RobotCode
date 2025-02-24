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
                                                                () -> driverXbox.getLeftY() * -1,
                                                                () -> driverXbox.getLeftX() * -1)
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

    /* TEST CONTROLS
      * A = GRAB ALGEA
      * B = RELEASE ALGEA
      * 
      * X = ARM LENGTH UP
      * Y = ARM LENGTH DOWN
      * 
      * Left Bump = ARM TILT BACKWARDS
      * Right Bump = ARM TILT FORWARD
      *
      * START = Zero Gyro
      * BACK = No Command
      */
    driverXbox.start().whileTrue(Commands.runOnce(drivebase::zeroGyro));
    driverXbox.a().whileTrue(Commands.startEnd(handtilt::close, handtilt::halt, handtilt));
    driverXbox.b().whileTrue(Commands.startEnd(handtilt::open, handtilt::halt,  handtilt));
    driverXbox.x().whileTrue(Commands.startEnd(armlength::Up, armlength::Halt, armlength));
    driverXbox.y().whileTrue(Commands.startEnd(armlength::Down, armlength::Halt, armlength));
    driverXbox.leftBumper().whileTrue(Commands.startEnd(armtilt::up, armtilt::halt, armtilt));
    driverXbox.rightBumper().whileTrue(Commands.startEnd(armtilt::down, armtilt::halt, armtilt));

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
      * BACK = None
    */
    // driverXbox.a().whileTrue(Commands.runOnce(drivebase::zeroGyro, drivebase));
    // driverXbox.b().whileTrue(drivebase.driveToPose(new Pose2d(new Translation2d(0, 0), Rotation2d.fromDegrees(180.0))));
    // driverXbox.x().whileTrue(Commands.runOnce(drivebase::lock, drivebase).repeatedly());
    // driverXbox.y().whileTrue(Commands.none());
    // driverXbox.leftBumper().whileTrue(Commands.startEnd(armtilt::up, armtilt::halt, armtilt));  //MIGHT HAVE TO SWAP THIS ONE WITH RIGHT
    // driverXbox.rightBumper().whileTrue(Commands.startEnd(armtilt::down, armtilt::halt, armtilt));
    // driverXbox.start().whileTrue(Commands.none());
    // driverXbox.back().whileTrue(Commands.none());
  }

  public void configureBindingsOper()
  {
    /* TESTING
     * A = CAPTURE CORAL
     * B = RELEASE CORAL
     * 
     * X = ARM EXTEND IN
     * Y = ARM EXTEND OUT
     * 
     * Left Bump = HAND TILT DOWN
     * Right Bump = HAND TILT UP
     *
     * START = Zero Gyro
     * BACK = No Command
     */
    // FLYWHEEL CAPTURE CORAL
    armXbox.a().whileTrue((Commands.startEnd(flywheel::in, flywheel::stop, flywheel).until(coraltrigger)));
    // FLYWHEEL RELEASE CORAL
    armXbox.b().whileTrue((Commands.startEnd(flywheel::out, flywheel::stop, flywheel)));

    //TILT HAND UPWARDS
    armXbox.leftTrigger().whileTrue((Commands.startEnd(handtilt::up, handtilt::stop, handtilt)));
    // TILT HAND DOWNWARDS
    armXbox.rightTrigger().whileTrue((Commands.startEnd(handtilt::down, handtilt::stop, handtilt)));

    // EXTEND OUTWARDS
    armXbox.y().whileTrue((Commands.startEnd(armExtend::out, armExtend::stop, armExtend)));
    // EXTEND INWARDS
    armXbox.x().whileTrue((Commands.startEnd(armExtend::in, armExtend::stop, armExtend)));

    /* COMPETITION PRACTICE
     * A = CAPTURE CORAL
     * B = RELEASE CORAL
     * 
     * X = CAPTURE ALGEA
     * Y = RELEASE ALGEA
     * 
     * Left Bump = None
     * Right Bump = None
     *
     * START = CANCEL ARM COMMANDS
     * BACK = MOVE TO START POSITION
     * 
     * DPAD DOWN = LOW REEF CORAL
     * DPAD DOWN RIGHT = !!!
     * DPAD RIGHT = MID REEF CORAL
     * DPAD UP RIGHT = !!!
     * DPAD UP = TOP REEF CORAL
     * DPAD UP LEFT = !!!
     * DPAD LEFT = HOME/HUMAN PLAYER POSITION
     * DPAD CENTER = MAKE THIS DEFAULT HOME???
     */
    armXbox.a().onTrue((Commands.startEnd(flywheel::in, flywheel::stop, flywheel).until(coraltrigger)));
      // coraltrigger.onTrue(Commands.none()); IF WE ARENT PICKING FROM GROUND THIS IS USELESS
      // coraltrigger.onFalse(Commands.runOnce(flywheel::stop, flywheel));
    armXbox.b().whileTrue(Commands.startEnd(flywheel::out, flywheel::stop, flywheel));
    armXbox.x().whileTrue(Commands.startEnd(handtilt::close, handtilt::hold, handtilt));
    armXbox.y().whileTrue(Commands.startEnd(handtilt::open, handtilt::stop, handtilt));
    // armXbox.povCenter().onTrue(
    //   new ParallelCommandGroup(
    //     new InstantCommand(() -> {handtilt.setSmartPosition(1);}, handtilt),
    //     new InstantCommand(() -> {armExtend.setSmartPosition(1);}, armExtend),
    //     new InstantCommand(() -> {armlength.setSmartPosition(1);}, armlength),
    //     new InstantCommand(() -> {armtilt.setSmartPosition(1);}, armtilt)
    //   ));
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
    armXbox.povRight().onTrue(
      new ParallelCommandGroup(
        new InstantCommand(() -> {handtilt.setSmartPosition(4);}, handtilt),
        new InstantCommand(() -> {armExtend.setSmartPosition(4);}, armExtend),
        new InstantCommand(() -> {armlength.setSmartPosition(4);}, armlength),
        new InstantCommand(() -> {armtilt.setSmartPosition(4);}, armtilt)
      ));
    armXbox.povUp().onTrue(
      new ParallelCommandGroup(
        new InstantCommand(() -> {handtilt.setSmartPosition(5);}, handtilt),
        new InstantCommand(() -> {armExtend.setSmartPosition(5);}, armExtend),
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
    return drivebase.getAutonomousCommand("New Auto");
  }

  public void setMotorBrake(boolean brake)
  {
    drivebase.setMotorBrake(brake);
  }

}


      /* DRIVER 0
       * A = GRAB ALGEA
       * B = RELEASE ALGEA
       * 
       * X = ARM LENGTH UP
       * Y = ARM LENGTH DOWN
       * 
       * Left Bump = ARM TILT BACKWARDS
       * Right Bump = ARM TILT FORWARD
       *
       * START = Zero Gyro
       * BACK = No Command
       */

     /* DRIVER 1
      * A = CAPTURE CORAL
      * B = RELEASE ALGEA
      * 
      * X = ARM EXTEND IN
      * Y = ARM EXTEND OUT
      * 
      * Left Bump = HAND TILT DOWN
      * Right Bump = HAND TILT UP
      *
      * START = Zero Gyro
      * BACK = No Command
      * DPADS = SEE ABOVE
      */