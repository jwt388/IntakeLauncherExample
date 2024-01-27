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

    public static final double POUND_IN2_TO_KG_METERS2 =
        Units.lbsToKilograms(1) * Math.pow(Units.inchesToMeters(1), 2);

    public static final double LAUNCHER_MOI_IN_LBS2 = 4.5;
    public static final double LAUNCHER_MOI_KG_METERS2 =
        LAUNCHER_MOI_IN_LBS2 * POUND_IN2_TO_KG_METERS2;
  }
}
