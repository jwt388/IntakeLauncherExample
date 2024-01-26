package frc.sim;

import edu.wpi.first.math.util.Units;

/** Constants utility class for the simulation. */
public final class Constants {

  private Constants() {
    throw new IllegalStateException("Utility class");
  }

  /** Elevator simulation constants. */
  public static final class IntakeLauncherSimConstants {
    private IntakeLauncherSimConstants() {
      throw new IllegalStateException("IntakeLauncherSimConstants Utility Class");
    }

    public static final double LAUNCHER_TORQUE_NM = 1.0;
    public static final double ELEVATOR_DRUM_RADIUS = Units.inchesToMeters(2.0);
    public static final double CARRIAGE_MASS = 4.0; // kg
  }
}
