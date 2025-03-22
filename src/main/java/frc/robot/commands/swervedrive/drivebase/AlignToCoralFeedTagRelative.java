package frc.robot.commands.swervedrive.drivebase;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;

public class AlignToCoralFeedTagRelative extends Command {
  private PIDController yController;
  private Timer dontSeeTagTimer, stopTimer;
  private SwerveSubsystem drivebase;
  private double tagID = -1;
  private double ty = 0.0;

  public AlignToCoralFeedTagRelative(boolean isRightFeed, SwerveSubsystem drivebase) {
    // xController = new PIDController(Constants.DrivebaseConstants.X_FEED_ALIGNMENT_P, 0.0, 0);  // Vertical movement (P,I,D)
    yController = new PIDController(Constants.DrivebaseConstants.Y_FEED_ALIGNMENT_P, 0.0, 0);  // Horitontal movement (P,I,D)
    // rotController = new PIDController(Constants.DrivebaseConstants.ROT_FEED_ALIGNMENT_P, 0.0, 0);  // Rotation (P,I,D)
    this.drivebase = drivebase;
    addRequirements(drivebase);
  }

  @Override
  public void initialize() {
    this.stopTimer = new Timer();
    this.stopTimer.start();
    this.dontSeeTagTimer = new Timer();
    this.dontSeeTagTimer.start();

    // rotController.setSetpoint(Constants.DrivebaseConstants.ROT_SETPOINT_FEED_ALIGNMENT);
    // rotController.setTolerance(Constants.DrivebaseConstants.ROT_TOLERANCE_FEED_ALIGNMENT);

    // xController.setSetpoint(Constants.DrivebaseConstants.X_SETPOINT_FEED_ALIGNMENT);
    // xController.setTolerance(Constants.DrivebaseConstants.X_TOLERANCE_FEED_ALIGNMENT);

    yController.setSetpoint(Constants.DrivebaseConstants.Y_SETPOINT_FEED_ALIGNMENT);
    yController.setTolerance(Constants.DrivebaseConstants.Y_TOLERANCE_FEED_ALIGNMENT);

    tagID = SmartDashboard.getNumber("tid", tagID);
  }

  @Override
  public void execute() {
    if (SmartDashboard.getNumber("tid", tagID) >=0) {
      this.dontSeeTagTimer.reset();
      ty = SmartDashboard.getNumber("ty", 0.0);
    };
      
      double ySpeed = -yController.calculate(ty);
      SmartDashboard.putNumber("CAMERA LL y", ty);

      drivebase.drive(new Translation2d(0.0, ySpeed), 0.0, false);

      if (!yController.atSetpoint()) {
        stopTimer.reset();
    } else {
      drivebase.drive(new Translation2d(), 0, false);
    }

    SmartDashboard.putNumber("poseValidTimer", stopTimer.get());
  }

  @Override
  public void end(boolean interrupted) {
    drivebase.drive(new Translation2d(), 0.0, false);
  }

  @Override
  public boolean isFinished() {
    // Requires the robot to stay in the correct position for 0.3 seconds, as long as it gets a tag in the camera
    return this.dontSeeTagTimer.hasElapsed(Constants.DrivebaseConstants.DONT_SEE_TAG_WAIT_TIME) ||
        stopTimer.hasElapsed(Constants.DrivebaseConstants.POSE_VALIDATION_TIME);
  }
}
