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
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
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

  // Replace with CommandPS4Controller or CommandJoystick if needed
  final         CommandXboxController driverXbox = new CommandXboxController(0);
  final         CommandXboxController armXbox = new CommandXboxController(1);
  // The robot's subsystems and commands are defined here...
  private final SwerveSubsystem       drivebase  = new SwerveSubsystem(new File(Filesystem.getDeployDirectory(),
                                                                                "swerve"));
  private final ArmLength armlength = new ArmLength();
  private final ArmTilt armtilt = new ArmTilt();
  private final ArmExtend armExtend = new ArmExtend();
  private final HandTilt handtilt = new HandTilt();
  private final FlyWheel flywheel = new FlyWheel();

  //* TRIGGERS */
  private final Trigger coraltrigger = new Trigger(flywheel::getSwitch);
  private final Trigger handtiltTrigger = new Trigger(handtilt::getswitch);
  private final Trigger armbasetrigger = new Trigger(armtilt::getbottomswitch);
  private final Trigger armtoptrigger = new Trigger(armlength::gettopswitch);
  private final Trigger armextendtrigger = new Trigger(armExtend::getSwitch);


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
    // configureBindingsOper();

    DriverStation.silenceJoystickConnectionWarning(true);
    // NamedCommands.registerCommand("test", Commands.print("I EXIST"));
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

    Command driveFieldOrientedDirectAngle         = drivebase.driveFieldOriented(driveDirectAngle);
    Command driveFieldOrientedAnglularVelocity    = drivebase.driveFieldOriented(driveAngularVelocity);
    Command driveSetpointGen                      = drivebase.driveWithSetpointGeneratorFieldRelative(driveDirectAngle);
    Command driveFieldOrientedDirectAngleSim      = drivebase.driveFieldOriented(driveDirectAngleSim);
    Command driveFieldOrientedAnglularVelocitySim = drivebase.driveFieldOriented(driveAngularVelocitySim);
    Command driveSetpointGenSim = drivebase.driveWithSetpointGeneratorFieldRelative(
        driveDirectAngleSim);

    // SET THE DRIVE TYPE
    drivebase.setDefaultCommand(driveFieldOrientedAnglularVelocity);


    if (DriverStation.isTest())
    {
      drivebase.setDefaultCommand(driveFieldOrientedAnglularVelocity); // Overrides drive command above!

      driverXbox.x().whileTrue(Commands.runOnce(drivebase::lock, drivebase).repeatedly());
      driverXbox.y().whileTrue(drivebase.driveToDistanceCommand(1.0, 0.2));
      driverXbox.start().onTrue((Commands.runOnce(drivebase::zeroGyro)));
      driverXbox.back().whileTrue(drivebase.centerModulesCommand());
      driverXbox.leftBumper().onTrue(Commands.none());
      driverXbox.rightBumper().onTrue(Commands.none());
    } else
    {
      /*
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
      driverXbox.b().onTrue(Commands.startEnd(handtilt::open, handtilt::halt,  handtilt));
      driverXbox.x().onTrue(Commands.startEnd(armlength::Up, armlength::Halt, armlength));
      driverXbox.y().onTrue(Commands.startEnd(armlength::Down, armlength::Halt, armlength).until(armtoptrigger));
      driverXbox.leftBumper().whileTrue(Commands.startEnd(armtilt::up, armtilt::halt, armtilt).until(armbasetrigger));
      driverXbox.rightBumper().onTrue(Commands.startEnd(armtilt::down, armtilt::halt, armtilt));

      // driverXbox.x().onTrue(Commands.runOnce(drivebase::addFakeVisionReading));
      // driverXbox.x().onTrue(Commands.none());
      // driverXbox.leftBumper().whileTrue(Commands.runOnce(drivebase::lock, drivebase).repeatedly());
      // driverXbox.back().whileTrue(Commands.none());
      // driverXbox.b().whileTrue(
      //     drivebase.driveToPose(
      //         new Pose2d(new Translation2d(4, 4), Rotation2d.fromDegrees(0)))
      // );
    }

  }

  public void configureBindingsOper()
  {
    // SET DEFAULT COMMAND FOR ARM LENGTHS
    // armlength.setDefaultCommand(Commands.run(() -> armlength.setbasespeed(armXbox.getRightY()), armlength));
    // armlength.setDefaultCommand(Commands.run(() -> armlength.settopspeed(armXbox.getLeftY()), armlength));
    
    /*
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
     */
    // FLYWHEEL CAPTURE CORAL
    armXbox.a().whileTrue((Commands.startEnd(flywheel::in, flywheel::stop, flywheel).until(coraltrigger)));
    // FLYWHEEL RELEASE CORAL
    armXbox.b().whileTrue((Commands.startEnd(flywheel::out, flywheel::stop, flywheel)));

    //TILT HAND UPWARDS
    armXbox.rightTrigger().whileTrue((Commands.startEnd(handtilt::up, handtilt::stop, handtilt)));
    // TILT HAND DOWNWARDS
    armXbox.leftTrigger().whileTrue((Commands.startEnd(handtilt::down, handtilt::stop, handtilt).until(handtiltTrigger)));

    // EXTEND OUTWARDS
    armXbox.y().whileTrue((Commands.startEnd(armExtend::out, armExtend::stop, armExtend)));
    // EXTEND INWARDS
    armXbox.x().whileTrue((Commands.startEnd(armExtend::in, armExtend::stop, armExtend).until(armextendtrigger)));


    // armXbox.back().onTrue(
    //     Commands.SequentialCommandGroup(
    //       Commands.InstantCommand(() -> handtilt.setTargetPosition(kposition.setpoint[0][3]), handtilt),
    //       Commands.InstantCommand(() -> armExtend.setTargetPosition(kposition.setpoint[0][2]), armExtend),
    //       Commands.InstantCommand(() -> armtilt.setTargetPosition(kposition.setpoint[0][1]), armtilt),
    //       Commands.InstantCommand(() -> armlength.setTargetPosition(kposition.setpoint[0][0]), armlength)
    //     )
    // );
    // MOVE ARM BASE FORWARD AND BACKWARDS
    // armXbox.start().and(armXbox.leftBumper()).whileTrue((Commands.startEnd(armtilt::up, armtilt::stop, armtilt)));
    // armXbox.start().and(armXbox.rightBumper()).whileTrue((Commands.startEnd(armtilt::down, armtilt::stop, armtilt)));




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



  public double deadbandDriveX() {
    return 0.0;
  }

  public double deadbandDriveY() {
    return 0.0;
  }

  public double deadbandDriveZ() {
    return 0.0;
}
}
