// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.RobotPreferences.PreferenceKeyValue;

public final class Constants {

  private Constants() {
    throw new IllegalStateException("Utility class");
  }

  // Run time options

  // Set to true to log Joystick data. To false otherwise.
  public static final boolean LOG_JOYSTICK_DATA = true;

  // Set to true to send telemetry data to Live Window. To false
  // to disable it.
  public static final boolean LW_TELEMETRY_ENABLE = false;

  public static final boolean LOOP_TIMING_LOG = false;

  // Set to true to log each frame of command execution. To false to disable.
  public static final boolean COMMAND_EXECUTE_LOG = false;

  /** Constants used for the Launcher subsystem. */
  public static final class IntakeConstants {

    private IntakeConstants() {
      throw new IllegalStateException("IntakeConstants Utility Class");
    }

    public static final int INTAKE_MOTOR_PORT = 8;
    public static final double INTAKE_GEAR_RATIO =
        1.0; // Ratio of motor rotations to output rotations
    public static final double INTAKE_COMMAND_VOLTS = 12.0;
  }

  /** Constants used for the Launcher subsystem. */
  public static final class LauncherConstants {

    private LauncherConstants() {
      throw new IllegalStateException("LauncherConstants Utility Class");
    }

    public static final int LAUNCHER_MOTOR_PORT = 9;

    // These are fake gains; in actuality these must be determined individually for each robot
    // Constants tunable through preferences
    public static final PreferenceKeyValue LAUNCHER_KP =
        new PreferenceKeyValue("LauncherKP", 6.0d / 1000);
    public static final PreferenceKeyValue LAUNCHER_KS_VOLTS =
        new PreferenceKeyValue("LauncherKS", 0.0);
    public static final PreferenceKeyValue LAUNCHER_KV_VOLTS_PER_RPM =
        new PreferenceKeyValue("LauncherKV", 6.0d / 1000);
    public static final PreferenceKeyValue LAUNCHER_KA_VOLTS_PER_RPM2 =
        new PreferenceKeyValue("LauncherKA", 0.0d / 1000);

    private static final PreferenceKeyValue[] LAUNCHER_PREFERENCES = {
      LAUNCHER_KP, LAUNCHER_KS_VOLTS, LAUNCHER_KV_VOLTS_PER_RPM, LAUNCHER_KA_VOLTS_PER_RPM2
    };

    public static PreferenceKeyValue[] getLauncherPreferences() {
      return LAUNCHER_PREFERENCES;
    }

    public static final double LAUNCHER_GEAR_RATIO =
        3.0; // Ratio of motor rotations to output rotations
    public static final double LAUNCHER_ROTATIONS_PER_ENCODER_ROTATION = 1.0d / LAUNCHER_GEAR_RATIO;
    public static final double LAUNCHER_TOLERANCE_RPM = 20;
    public static final double LAUNCHER_FULL_SPEED = 600;
  }

  /** Constants used for assigning operator input. */
  public static final class OIConstants {

    private OIConstants() {
      throw new IllegalStateException("OIConstants Utility Class");
    }

    public static final int OPERATOR_CONTROLLER_PORT = 0;
  }
}
