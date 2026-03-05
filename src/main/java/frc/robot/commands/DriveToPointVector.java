package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.drive.Drive;
import org.littletonrobotics.junction.Logger;

public class DriveToPointVector extends Command {

  private final Drive drive;
  private final Pose2d target;

  private final PIDController thetaController = new PIDController(4.0, 0, 0);

  private static final double kP_distance = 2.2;

  private double MAX_SPEED; // m/s
  private static final double POSITION_TOLERANCE = 0.10; // meters
  private static final double ANGLE_TOLERANCE = Math.toRadians(5);


  private static final double MIN_SPEED = 0.25;    // m/s
  private static final double SLOW_RADIUS = 1.0;   // meters
  private static final double STOP_RADIUS = 0.12; // meters

  public DriveToPointVector(Drive drive, Pose2d target, double max_speed) { // 3 m/s is a good starting point
    this.drive = drive;
    this.target = target;
    this.MAX_SPEED = max_speed;

    thetaController.enableContinuousInput(-Math.PI, Math.PI);

    addRequirements(drive);
  }

  @Override
  public void initialize() {
    thetaController.reset();
  }

  @Override
  public void execute() {

    Pose2d pose = drive.getPose();

    Translation2d current = pose.getTranslation();
    Translation2d goal = target.getTranslation();


    
    Translation2d errorVector = goal.minus(current);

    double distance = errorVector.getNorm();

    

    Logger.recordOutput("DriveToPoint/TargetPose", target);
    Logger.recordOutput("DriveToPoint/DistanceError", distance);


    Translation2d direction =
        distance > 0
            ? errorVector.div(distance)
            : new Translation2d();


    double angleError =
        Math.abs(
            pose.getRotation()
                .minus(target.getRotation())
                .getRadians());

    if ( distance < POSITION_TOLERANCE
        && angleError < ANGLE_TOLERANCE ) { // don't start moving if we're already there. 
            drive.stop();
            return;
        }

    double speed = kP_distance * distance;

    // clamp to max
    speed = Math.min(speed, MAX_SPEED);

    // slow down smoothly near the target
    if (distance < SLOW_RADIUS) {
    speed *= distance / SLOW_RADIUS;
    }

    // ensure we don't stall when far away
    speed = Math.max(speed, MIN_SPEED);

    Translation2d velocity = direction.times(speed);

    double omega =
        thetaController.calculate(
            pose.getRotation().getRadians(),
            target.getRotation().getRadians());

    double headingError =
        Math.abs(
            pose.getRotation()
                .minus(target.getRotation())
                .getRadians());

    double headingScale =
        Math.max(0.2, 1.0 - (headingError / Math.PI));

    speed *= headingScale;

    Logger.recordOutput(
        "DriveToPoint/TargetHeading",
        target.getRotation().getDegrees()
    );
    Logger.recordOutput("DriveToPoint/DriveCommandSpeed", speed);

    drive.runVelocity(
        ChassisSpeeds.fromFieldRelativeSpeeds(
            velocity.getX(),
            velocity.getY(),
            omega,
            pose.getRotation()));
  }

  @Override
  public boolean isFinished() {

    Pose2d pose = drive.getPose();

    double distance =
        pose.getTranslation()
            .getDistance(target.getTranslation());

    double angleError =
        Math.abs(
            pose.getRotation()
                .minus(target.getRotation())
                .getRadians());

    return distance < POSITION_TOLERANCE
        && angleError < ANGLE_TOLERANCE;
  }

  @Override
  public void end(boolean interrupted) {
    drive.stop();
  }
}