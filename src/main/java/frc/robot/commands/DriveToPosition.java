package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;

import frc.robot.subsystems.drive.Drive;

public class DriveToPosition extends Command {

  private final Drive drive;
  private final Pose2d target;

  private final PIDController xController = new PIDController(2.5, 0, 0);
  private final PIDController yController = new PIDController(2.5, 0, 0);
  private final PIDController thetaController = new PIDController(4.0, 0, 0);

  public DriveToPosition(Drive drive, Pose2d target) {
    this.drive = drive;
    this.target = target;

    thetaController.enableContinuousInput(-Math.PI, Math.PI);

    addRequirements(drive);
  }

  @Override
  public void initialize() {
    xController.reset();
    yController.reset();
    thetaController.reset();
  }

  @Override
  public void execute() {

    Pose2d pose = drive.getPose();

    double vx =
        xController.calculate(
            pose.getX(),
            target.getX());

    double vy =
        yController.calculate(
            pose.getY(),
            target.getY());

    double omega =
        thetaController.calculate(
            pose.getRotation().getRadians(),
            target.getRotation().getRadians());

    drive.runVelocity(
        ChassisSpeeds.fromFieldRelativeSpeeds(
            vx,
            vy,
            omega,
            pose.getRotation()));
  }

  @Override
  public boolean isFinished() {

    Pose2d pose = drive.getPose();

    double dx = Math.abs(pose.getX() - target.getX());
    double dy = Math.abs(pose.getY() - target.getY());
    double dtheta =
        Math.abs(
            pose.getRotation()
                .minus(target.getRotation())
                .getRadians());

    return dx < 0.1 && dy < 0.1 && dtheta < Math.toRadians(5);
  }

  @Override
  public void end(boolean interrupted) {
    drive.stop();
  }
}