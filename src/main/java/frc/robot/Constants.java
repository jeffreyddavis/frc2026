// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.RobotBase;

/**
 * This class defines the runtime mode used by AdvantageKit. The mode is always "real" when running
 * on a roboRIO. Change the value of "simMode" to switch between "sim" (physics sim) and "replay"
 * (log replay from a file).
 */
public final class Constants {
  public static final Mode simMode = Mode.SIM;
  public static final Mode currentMode = RobotBase.isReal() ? Mode.REAL : simMode;

  public static final boolean disableHAL = false;

  public static final double YawWarningTolerance = 2;
  public static final double TrenchDangerDistance = 2;
  public static final double ZONE_HYSTERESIS = 0.15; // meters (~6 in)

  public static final double MAX_SPEED = Units.feetToMeters(14.5);

  public static enum Mode {
    /** Running on a real robot. */
    REAL,

    /** Running a physics simulator. */
    SIM,

    /** Replaying from a log file. */
    REPLAY
  }

  // ===============================
  // CANivore Bus (Drivetrain Only)
  // ===============================
  public static final class Drivetrain {
    // Front Left Module
    public static final int FL_Drive = 1;
    public static final int FL_Steer = 2;
    public static final int FL_Encoder = 3;

    // Front Right Module
    public static final int FR_Drive = 4;
    public static final int FR_Steer = 5;
    public static final int FR_Encoder = 6;

    // Back Left Module
    public static final int BL_Drive = 7;
    public static final int BL_Steer = 8;
    public static final int BL_Encoder = 9;

    // Back Right Module
    public static final int BR_Drive = 10;
    public static final int BR_Steer = 11;
    public static final int BR_Encoder = 12;

    // Gyro
    public static final int Pigeon = 13;

    // Canivore
    public static final int Canivore = 14; // Also 0 sometimes.
  }

  public static final class SystemDevices {
    public static final int PDH = 20;
    public static final int RoboRIO = 21; // Documentation reference only
  }

  public static final class Turret {
    public static final int Motor = 30;
    public static final int Encoder = 31;
    public static final double TestSpeed = .1;
    public static final double EncoderOffset = 0;
    public static final double FORBIDDEN_LIMIT_DEG = 180.0;
    public static final double FORBIDDEN_BUFFER_DEG = 5.0; // safety margin
    public static final double GEAR_RATIO = 10.0; // example
    public static final boolean HardwareEnabled = true;

    public static final double kP = .007;
    public static final double kS = .2;
    public static final double maxOutput = .25;
    public static final double toleranceDeg = .1;
    public static final double OMEGA_LOOKAHEAD = .26;
    ;
  }

  public static final class Loader {
    public static final int Motor = 40;
    public static final boolean HardwareEnabled = false;
  }

  public static final class Hood {
    public static final boolean HardwareEnabled = true;
    public static final double distanceToleranceMM = 1;
  }

  public static final class Spindexer {
    public static final int Motor = 50;
    public static final boolean HardwareEnabled = true;
  }

  public static final class Intake {
    public static final int ArmLeader = 60;
    public static final int ArmFollower = 61;
    public static final int ArmEncoder = 55;
    public static final int RollerLeft = 53;
    public static final int RollerRight = 54;
    public static final boolean HardwareEnabled = true;
  }

  public static final class Shooter {
    public static final int LeftMotor = 36;
    public static final int RightMotor = 35;
    public static final double PassRPM = 3000.0;
    public static final double AutoRPM = 3500.0;
    public static final double RPSTolerance = 5.0;
    public static final boolean HardwareEnabled = true;
  }
}
