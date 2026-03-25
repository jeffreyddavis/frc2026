// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot;

import static frc.robot.subsystems.vision.VisionConstants.*;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import frc.robot.commands.*;
import frc.robot.generated.TunerConstants;
import frc.robot.shot.ShotController;
import frc.robot.subsystems.*;
import frc.robot.subsystems.drive.*;
import frc.robot.subsystems.vision.*;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // Subsystems
  public final Drive drive;
  private final Vision vision;
  private final ShotController shotController;

  // Controller
  private final CommandXboxController controller = new CommandXboxController(1);
  private final Joystick stick = new Joystick(0);

  // Buttons
  private final JoystickButton trigger = new JoystickButton(stick, 1);
  private final JoystickButton EnableAuto = new JoystickButton(stick, 2);

  private final JoystickButton intakeIn = new JoystickButton(stick, 5);
  private final JoystickButton intakeReverse = new JoystickButton(stick, 6);

  private final JoystickButton deployArm = new JoystickButton(stick, 3);
  private final JoystickButton retractArm = new JoystickButton(stick, 4);

  private final JoystickButton manualSpindexer = new JoystickButton(stick, 7);
  private final JoystickButton agitateIntake = new JoystickButton(stick, 14);

  private final JoystickButton resetTurret = new JoystickButton(stick, 11);

  private final POVButton hoodUp = new POVButton(stick, 0);
  private final POVButton hoodDown = new POVButton(stick, 180);

  private final POVButton turretRight = new POVButton(stick, 90);
  private final POVButton turretLeft = new POVButton(stick, 270);

  private final JoystickButton shooterEnable = new JoystickButton(stick, 9);
  private final JoystickButton passMode = new JoystickButton(stick, 10);

  private final JoystickButton resetGyro = new JoystickButton(stick, 8);

  private final ShootingCoordinator coordinator;
  private final Shooter shooter;
  private final Turret turret;
  private final Hood hood;
  public final Intake intake;
  private final Loader loader;
  private final Spindexer spindexer;
  public final QuestNavSub m_QuestNav;

  // Dashboard inputs
  private final LoggedDashboardChooser<Command> autoChooser;

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {

    turret = new Turret();
    switch (Constants.currentMode) {
      case REAL:
        drive =
            new Drive(
                new GyroIOPigeon2(),
                new ModuleIOTalonFX(TunerConstants.FrontLeft),
                new ModuleIOTalonFX(TunerConstants.FrontRight),
                new ModuleIOTalonFX(TunerConstants.BackLeft),
                new ModuleIOTalonFX(TunerConstants.BackRight));

        vision =
            new Vision(
                drive::addVisionMeasurement,
                turret,
                drive,
                new VisionIOLimelight(camera0Name, drive::getRotation),
                new VisionIOLimelight(camera1Name, drive::getRotation),
                new VisionIOLimelight(camera2Name, drive::getRotation));

        break;

      case SIM:
        // Sim robot, instantiate physics sim IO implementations
        drive =
            new Drive(
                new GyroIO() {},
                new ModuleIOSim(TunerConstants.FrontLeft),
                new ModuleIOSim(TunerConstants.FrontRight),
                new ModuleIOSim(TunerConstants.BackLeft),
                new ModuleIOSim(TunerConstants.BackRight));

        vision =
            new Vision(
                drive::addVisionMeasurement, turret, drive, new VisionIO() {}, new VisionIO() {});
        break;

      default:
        // Replayed robot, disable IO implementations
        drive =
            new Drive(
                new GyroIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                new ModuleIO() {});
        vision =
            new Vision(
                drive::addVisionMeasurement, turret, drive, new VisionIO() {}, new VisionIO() {});
        break;
    }

    // Set up SysId routines

    // autoChooser.addOption(
    //    "Drive Wheel Radius Characterization", DriveCommands.wheelRadiusCharacterization(drive));
    // autoChooser.addOption(
    //    "Drive Simple FF Characterization", DriveCommands.feedforwardCharacterization(drive));
    //   autoChooser.addOption(
    //       "Drive SysId (Quasistatic Forward)",
    //       drive.sysIdQuasistatic(SysIdRoutine.Direction.kForward));
    //    autoChooser.addOption(
    //       "Drive SysId (Quasistatic Reverse)",
    //      drive.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));
    //  autoChooser.addOption(
    //      "Drive SysId (Dynamic Forward)", drive.sysIdDynamic(SysIdRoutine.Direction.kForward));
    //  autoChooser.addOption(
    ///      "Drive SysId (Dynamic Reverse)", drive.sysIdDynamic(SysIdRoutine.Direction.kReverse));

    m_QuestNav = new QuestNavSub(drive);

    // Instantiate subsystems
    shooter = new Shooter();

    hood = new Hood();
    intake = new Intake();
    loader = new Loader();
    spindexer = new Spindexer();
    shotController = new ShotController();
    shotController.forceDisableTuning();

    RobotHealth robotHealth = new RobotHealth(drive, m_QuestNav, vision);

    coordinator =
        new ShootingCoordinator(
            shooter, turret, hood, loader, spindexer, robotHealth, shotController, drive);

    // Configure the button bindings
    configureButtonBindings();
    addNamedCommands();

    // Set up auto routines
    autoChooser = new LoggedDashboardChooser<>("Auto Choices", AutoBuilder.buildAutoChooser());

    autoChooser.addOption(
        "Drive Wheel Radius Characterization", DriveCommands.wheelRadiusCharacterization(drive));
    autoChooser.addOption(
        "Drive Simple FF Characterization", DriveCommands.feedforwardCharacterization(drive));
  }

  private void addNamedCommands() {
    NamedCommands.registerCommand("StartShooter", new StartShooter(shooter));

    NamedCommands.registerCommand("Shoot", new ShootAuto(coordinator));

    NamedCommands.registerCommand("StopShoot", new StopShoot(loader, spindexer));

    NamedCommands.registerCommand("IntakeOut", new IntakeOut(intake));

    NamedCommands.registerCommand("IntakeIn", new IntakeIn(intake));

    NamedCommands.registerCommand("IntakeBarge", new IntakeBarge(intake));

    NamedCommands.registerCommand("AutoAimOn", new AutoAimOn(coordinator, spindexer));

    NamedCommands.registerCommand("PrepCloseShot", new PrepCloseShot(hood, shooter, spindexer));

    NamedCommands.registerCommand("WaitForShooterReady", new WaitForShooterReady(shooter, .5));

    NamedCommands.registerCommand(
        "ExpandAtMatchStart", new ExpandAtMatchStart(intake, hood, turret));
  }

  private void configureButtonBindings() {
    // Default command, normal field-relative drive
    drive.setDefaultCommand(
        DriveCommands.joystickDrive(
            drive,
            () -> -stick.getRawAxis(1),
            () -> -stick.getRawAxis(0),
            () -> -stick.getRawAxis(2) * .7));

    // Switch to X pattern when X button is pressed

    controller
        .x()
        .onTrue(Commands.runOnce(() -> intake.jogUp(), intake))
        .onFalse(Commands.runOnce(() -> intake.stopArm()));
    controller
        .y()
        .onTrue(Commands.runOnce(() -> intake.jogDown(), intake))
        .onFalse(Commands.runOnce(() -> intake.stopArm()));

    controller.a().onTrue(Commands.runOnce(() -> shooter.jogPercent(.01)));
    controller.b().onTrue(Commands.runOnce(() -> shooter.jogPercent(-.01)));

    controller.rightTrigger().onTrue(new ReadyMatchPosition(intake, hood, turret));
    controller.leftTrigger().onTrue(new ExpandAtMatchStart(intake, hood, turret));
    controller
        .pov(270)
        .whileTrue(
            new RunCommand(
                () -> {
                  coordinator.setMode(ShootingCoordinator.ShootingMode.MANUAL);
                  turret.jogLeft();
                  // coordinator.trimLeft();
                }));
    controller
        .pov(90)
        .whileTrue(
            new RunCommand(
                () -> {
                  // coordinator.trimRight();
                  coordinator.setMode(ShootingCoordinator.ShootingMode.MANUAL);
                  turret.jogRight();
                },
                turret));

    agitateIntake.onTrue(Commands.runOnce(() -> intake.startTimedAgitate(), intake));
    // resetGyro.onTrue(new StartShooter(shooter));

    // shooter.setDefaultCommand(Commands.runOnce(() -> shooter.disable(), shooter));
    // spindexer.setDefaultCommand(Commands.runOnce(() -> spindexer.feed(), spindexer));

    // intake.setDefaultCommand(Commands.run(() -> intake.goToArbitrayAngle(), intake));

    // loader.setDefaultCommand(Commands.runOnce(() -> loader.feed(), loader));
    /*
    turret.setDefaultCommand(
        Commands.runOnce(
            () ->
                turret.setFieldTargetAngle(
                    0,
                    drive.getRotation(),
                    Math.toDegrees(drive.getChassisSpeeds().omegaRadiansPerSecond)),
            turret)); */

    // Reset gyro to 0° when B button is pressed
    resetGyro.onTrue(
        Commands.runOnce(
                () -> {
                  if (FlipUtil.shouldFlip()) {
                    drive.setPose(new Pose2d(drive.getPose().getTranslation(), Rotation2d.k180deg));
                  } else {
                    drive.setPose(new Pose2d(drive.getPose().getTranslation(), Rotation2d.kZero));
                  }
                },
                drive)
            .ignoringDisable(true));

    resetTurret.onTrue(
        Commands.runOnce(
                () -> {
                  turret.zeroTurret();
                  turret.setTurretRelativeAngle(0);
                },
                turret)
            .ignoringDisable(true));

    /* ================= AUTO AIM MODE ================= */

    EnableAuto.onTrue(
        new InstantCommand(
            () -> {
              coordinator.setMode(ShootingCoordinator.ShootingMode.AUTO_AIM);
              spindexer.setFeedPercent(.25);
              // hood.retract();
              // shooter.setTargetRPM(0);
            }));

    /* ================= SHOOT REQUEST ================= */

    trigger.onTrue(
        new InstantCommand(
            () -> {
              coordinator.setRequestShot(true);
              // intake.startAgitate();
            }));

    trigger.onFalse(
        new InstantCommand(
            () -> {
              coordinator.setRequestShot(false);
              //  intake.stopAgitate();
            }));

    /* ================= INTAKE ================= */
    /*
       intakeIn
           .whileTrue(
               new RunCommand(
                   () -> {
                     intake.intake();
                     spindexer.feed();
                   }))
           .onFalse(
               new InstantCommand(
                   () -> {
                     intake.stopRollers();
                     spindexer.stop();
                   }));

       intakeReverse
           .whileTrue(
               new RunCommand(
                   () -> {
                     intake.outtake();
                     spindexer.reverse();
                   }))
           .onFalse(
               new InstantCommand(
                   () -> {
                     intake.stopRollers();
                     spindexer.stop();
                   }));
    */
    /* ================= ARM ================= */

    deployArm.onTrue(new InstantCommand(() -> intake.deploy()));

    retractArm.onTrue(new IntakeIn(intake));

    /* ================= MANUAL OVERRIDES ================= */
    /*
              manualSpindexer
                  .whileTrue(new RunCommand(() -> spindexer.feed()))
                  .onFalse(new InstantCommand(() -> spindexer.stop()));

              manualLoader
                  .whileTrue(new RunCommand(() -> loader.feed()))
                  .onFalse(new InstantCommand(() -> loader.stop()));
    */
    hoodUp.whileTrue(
        new RunCommand(
            () -> {
              // coordinator.setMode(ShootingMode.MANUAL);
              // hood.extendFully();
              coordinator.incrementUp();
              // hood.incrementUp();
            }));

    hoodDown.whileTrue(
        new RunCommand(
            () -> {
              // coordinator.setMode(ShootingMode.MANUAL);
              // hood.retract();
              coordinator.incrementDown();
              // hood.incrementDown();
            }));

    turretLeft.whileTrue(
        new RunCommand(
            () -> {
              // coordinator.setMode(ShootingMode.MANUAL);
              // turret.jogLeft();
              coordinator.trimLeft();
            }));

    turretRight.whileTrue(
        new RunCommand(
            () -> {
              coordinator.trimRight();
              // coordinator.setMode(ShootingMode.MANUAL);
              // turret.jogRight();
            },
            turret));

    shooterEnable.onTrue(new RunCommand(() -> shooter.enableClosedLoop()));
    /*
    passMode.onTrue(
        new InstantCommand(() -> coordinator.setMode(ShootingCoordinator.ShootingMode.AUTO_AIM))); */
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return autoChooser.get();
  }
}
