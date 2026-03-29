package org.littletonrobotics.frc2026;

import edu.wpi.first.wpilibj.RobotBase;

public final class Constants {
  public static final RobotType robot = RobotType.DIGGER;
  public static final boolean tuningMode = false;

  public static final double loopPeriodSecs = 0.02;
  public static final double loopPeriodWatchdogSecs = 0.2;

  public static Mode getMode() {
    return switch (robot) {
      case DIGGER -> RobotBase.isReal() ? Mode.REAL : Mode.REPLAY;
      case SIMBOT -> Mode.SIM;
    };
  }

  public enum Mode {
    /** Running on a real robot. */
    REAL,

    /** Running a physics simulator. */
    SIM,

    /** Replaying from a log file. */
    REPLAY
  }

  public enum RobotType {
    DIGGER,
    SIMBOT
  }

  public static boolean disableHAL = false;

  public static void disableHAL() {
    disableHAL = true;
  }

  /** Checks whether the correct robot is selected when deploying. */
  public static class CheckDeploy {
    public static void main(String... args) {
      if (robot == RobotType.SIMBOT) {
        System.err.println("Cannot deploy, invalid robot selected: " + robot);
        System.exit(1);
      }
    }
  }

  /** Checks that the default robot is selected and tuning mode is disabled. */
  public static class CheckPullRequest {
    public static void main(String... args) {
      if (robot != RobotType.DARWIN || tuningMode) {
        System.err.println("Do not merge, non-default constants are configured.");
        System.exit(1);
      }
    }
  }
}
