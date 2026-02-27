// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.RobotBase;

/**
 * This class defines the runtime mode used by AdvantageKit. The mode is always "real" when running
 * on a roboRIO. Change the value of "simMode" to switch between "sim" (physics sim) and "replay"
 * (log replay from a file).
 */
public final class Constants {
  public static final Mode simMode = Mode.SIM;
  public static final Mode currentMode = RobotBase.isReal() ? Mode.REAL : simMode;

  public static enum Mode {
    /** Running on a real robot. */
    REAL,

    /** Running a physics simulator. */
    SIM,

    /** Replaying from a log file. */
    REPLAY
  }

  public static final class Turret {
      public static final int Motor = 20;
      public static final int Encoder = 21;
      public static final double TestSpeed = .1;
      public static final double EncoderOffset = 0;
      public static final double FORBIDDEN_LIMIT_DEG = 180.0;
      public static final double FORBIDDEN_BUFFER_DEG = 5.0; // safety margin    
      public static final double GEAR_RATIO = 45.0; // example

  }

  public static final class Loader {
    public static final int Motor = 30;
  }

  public static final class Spindexer {
    public static final int Motor = 40;
  }

  public static final class Intake {
    public static final int ArmLeader = 50;
    public static final int ArmFollower = 51;
    public static final int RollerLeft = 52;
    public static final int RollerRight = 53;

  }

}
