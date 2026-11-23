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
import edu.wpi.first.wpilibj.DriverStation;
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
  // public final Vision vision;
  private final ShotController shotController;

  public boolean DefenseMode = false;

  // Controller
  private final CommandXboxController controller = new CommandXboxController(1);
  private final Joystick stick = new Joystick(0);

  // Buttons
  private final JoystickButton trigger = new JoystickButton(stick, 1);
  private final JoystickButton EnableAuto = new JoystickButton(stick, 3);

  //  private final JoystickButton intakeIn = new JoystickButton(stick, 5);
  // private final JoystickButton intakeReverse = new JoystickButton(stick, 6);

  private final JoystickButton deployArm = new JoystickButton(stick, 2);
  private final JoystickButton retractArm = new JoystickButton(stick, 4);

  // private final JoystickButton manualSpindexer = new JoystickButton(stick, 7);
  private final JoystickButton agitateIntake = new JoystickButton(stick, 14);

  // private final JoystickButton resetTurret = new JoystickButton(stick, 11);

  private final POVButton hoodUp = new POVButton(stick, 0);
  private final POVButton hoodDown = new POVButton(stick, 180);

  // private final POVButton turretRight = new POVButton(stick, 90);
  // private final POVButton turretLeft = new POVButton(stick, 270);

  private final JoystickButton startDefense = new JoystickButton(stick, 10);
  // private final JoystickButton passMode = new JoystickButton(stick, 10);

  private final JoystickButton resetGyro = new JoystickButton(stick, 8);

  // public final ShootingCoordinator coordinator;
  private final Shooter shooter;
  // private final Turret turret;
  private final Hood hood;
  public final Intake intake;
  private final Loader loader;
  private final Spindexer spindexer;
  public RobotHealth robotHealth;
  // public final QuestNavSub m_QuestNav;

  // Dashboard inputs
  private final LoggedDashboardChooser<Command> autoChooser;

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {

    // = new Turret();
    switch (Constants.currentMode) {
      case REAL:
        drive =
            new Drive(
                new GyroIOPigeon2(),
                new ModuleIOTalonFX(TunerConstants.FrontLeft),
                new ModuleIOTalonFX(TunerConstants.FrontRight),
                new ModuleIOTalonFX(TunerConstants.BackLeft),
                new ModuleIOTalonFX(TunerConstants.BackRight));

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

    // m_QuestNav = new QuestNavSub(drive);

    // Instantiate subsystems
    shooter = new Shooter();

    hood = new Hood();
    intake = new Intake();
    loader = new Loader();
    spindexer = new Spindexer();
    shotController = new ShotController();
    shotController.forceDisableTuning();

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

    NamedCommands.registerCommand("StopShoot", new StopShoot(loader, spindexer));

    NamedCommands.registerCommand("IntakeOut", new IntakeOut(intake));

    NamedCommands.registerCommand("IntakeIn", new IntakeIn(intake));

    NamedCommands.registerCommand("IntakeBarge", new IntakeBarge(intake));

    NamedCommands.registerCommand("PrepCloseShot", new PrepCloseShot(hood, shooter, spindexer));

    NamedCommands.registerCommand("WaitForShooterReady", new WaitForShooterReady(shooter, .5));

    NamedCommands.registerCommand("WaitForHoodReady", new WaitForHoodReady(hood, 2));
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

    controller.pov(270);
    controller.pov(90);

    controller
        .pov(0)
        .whileTrue(
            new RunCommand(
                () -> {
                  // coordinator.trimRight();
                  hood.incrementUp();
                },
                hood));
    controller
        .pov(180)
        .whileTrue(
            new RunCommand(
                () -> {
                  // coordinator.trimRight();
                  hood.incrementDown();
                },
                hood));

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

    /* ================= AUTO AIM MODE ================= */

    trigger
        .and(DriverStation::isTeleopEnabled)
        .whileTrue(Commands.startEnd(spindexer::feed, spindexer::stop, spindexer));

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
              // hood.incrementUp();
            }));

    hoodDown.whileTrue(
        new RunCommand(
            () -> {
              // coordinator.setMode(ShootingMode.MANUAL);
              // hood.retract();
              // hood.incrementDown();
            }));
    // shooterEnable.onTrue(new RunCommand(() -> shooter.enableClosedLoop()));
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
