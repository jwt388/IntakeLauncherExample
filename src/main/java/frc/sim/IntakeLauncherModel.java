// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.sim;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.BatterySim;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;
import frc.robot.Constants.IntakeLauncherConstants;
import frc.robot.subsystems.IntakeLauncherSubsystem;
import frc.sim.Constants.IntakeLauncherSimConstants;

/** A robot arm simulation based on a linear system model with Mech2d display. */
public class IntakeLauncherModel implements AutoCloseable {

  private final IntakeLauncherSubsystem intakeLauncherSubsystem;
  private double simLauncherCurrent = 0.0;
  private CANSparkMaxSim sparkSim;

  // The arm gearbox represents a gearbox containing one motor.
  private final DCMotor launcherGearbox = DCMotor.getNEO(1);

  /** Create a new ElevatorModel. */
  public IntakeLauncherModel(IntakeLauncherSubsystem intakeLauncherSubsystemToSimulate) {

    intakeLauncherSubsystem = intakeLauncherSubsystemToSimulate;
    simulationInit();

    // There is nothing to add to the dashboard for this sim since output is motor speed.
  }

  /** Initialize the arm simulation. */
  public void simulationInit() {

    // Setup a simulation of the CANSparkMax and methods to set values
    sparkSim = new CANSparkMaxSim(IntakeLauncherConstants.LAUNCHER_MOTOR_PORT);
  }

  /** Update the simulation model. */
  public void updateSim() {

    // Calculate the speed from the commanded motor voltage
    double simLauncherSpeed =
        launcherGearbox.getSpeed(
            IntakeLauncherSimConstants.LAUNCHER_TORQUE_NM,
            intakeLauncherSubsystem.getLauncherVoltageCommand());

    // Finally, we set our simulated encoder's readings and simulated battery voltage and
    // save the current so it can be retrieved later.
    sparkSim.setVelocity(simLauncherSpeed);
    simLauncherCurrent =
        launcherGearbox.getCurrent(1.0, intakeLauncherSubsystem.getLauncherVoltageCommand());
    sparkSim.setCurrent(simLauncherCurrent);

    // SimBattery estimates loaded battery voltages
    RoboRioSim.setVInVoltage(BatterySim.calculateDefaultBatteryLoadedVoltage(simLauncherCurrent));
  }

  /** Return the simulated current. */
  public double getSimCurrent() {
    return simLauncherCurrent;
  }

  @Override
  public void close() {
    // Add closeable objects here
  }
}
