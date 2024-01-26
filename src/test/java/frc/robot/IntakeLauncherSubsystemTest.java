// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static org.assertj.core.api.Assertions.assertThat;
import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.mockito.ArgumentMatchers.anyDouble;
import static org.mockito.Mockito.mock;
import static org.mockito.Mockito.times;
import static org.mockito.Mockito.verify;
import static org.mockito.Mockito.when;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.NetworkTableType;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.IntakeLauncherConstants;
import frc.robot.subsystems.IntakeLauncherSubsystem;
import java.util.HashMap;
import java.util.Map;
import java.util.Set;
import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.DisplayName;
import org.junit.jupiter.api.Test;
import org.mockito.AdditionalMatchers;

class IntakeLauncherSubsystemTest {
  private static final double DELTA = 5e-3;
  private Map<String, Double> telemetryDoubleMap = new HashMap<>();
  private Map<String, Boolean> telemetryBooleanMap = new HashMap<>();

  private IntakeLauncherSubsystem.Hardware launcherHardware;
  private IntakeLauncherSubsystem launcher;
  private CANSparkMax mockMotor;
  private RelativeEncoder mockEncoder;

  @BeforeEach
  public void initEach() {
    // Create mock hardware devices
    mockMotor = mock(CANSparkMax.class);
    mockEncoder = mock(RelativeEncoder.class);

    // Reset preferences to default values so test results are consistent
    RobotPreferences.resetPreferences();

    // Create subsystem object using mock hardware
    launcherHardware = new IntakeLauncherSubsystem.Hardware(mockMotor, mockEncoder);
    launcher = new IntakeLauncherSubsystem(launcherHardware);
  }

  @AfterEach
  public void closeLauncher() {
    launcher.close(); // motor is closed from the launcher close method
  }

  @Test
  @DisplayName("Test constructor and initialization.")
  void testConstructor() {
    // We haven't enabled it yet, so command to motor and saved value should be zero.
    verify(mockMotor).setVoltage(0.0);
    assertThat(launcher.getLauncherVoltageCommand()).isZero();
  }

  @Test
  @DisplayName("Test run command and stop.")
  void testMoveCommand() {

    // Create a command to run the launcher then initialize
    Command runLauncherCommand = launcher.runLauncher(IntakeLauncherConstants.LAUNCHER_FULL_SPEED);
    runLauncherCommand.initialize();

    // Run the periodic method to generate telemetry and verify it was published
    launcher.periodic();
    int numEntries = readTelemetry();
    assertThat(numEntries).isPositive();
    assertEquals(
        IntakeLauncherConstants.LAUNCHER_FULL_SPEED,
        telemetryDoubleMap.get("Launcher Setpoint"),
        DELTA);

    // Execute the command to run the controller
    runLauncherCommand.execute();
    launcher.periodic();
    readTelemetry();
    assertThat(telemetryDoubleMap.get("Launcher Voltage")).isPositive();
    assertThat(telemetryBooleanMap.get("Launcher Enabled")).isTrue();

    // When disabled mMotor should be commanded to zero
    launcher.disable();
    launcher.periodic();
    readTelemetry();
    verify(mockMotor, times(2)).setVoltage(0.0);
    assertThat(telemetryDoubleMap.get("Launcher Voltage")).isZero();
    assertThat(telemetryBooleanMap.get("Launcher Enabled")).isFalse();
  }

  @Test
  @DisplayName("Test Motor and Encoder Sensors.")
  void testSensors() {

    // Set values for mocked sensors
    final double fakeCurrent = -3.3;
    when(mockMotor.getOutputCurrent()).thenReturn(fakeCurrent);
    final double fakeVelocity = 123.5;
    when(mockEncoder.getVelocity()).thenReturn(fakeVelocity);

    // The motor voltage should be set twice: once to 0 when configured and once to a
    // positive value when controller is run.
    Command runLauncherCommand = launcher.runLauncher(IntakeLauncherConstants.LAUNCHER_FULL_SPEED);
    runLauncherCommand.initialize();
    runLauncherCommand.execute();
    verify(mockMotor, times(2)).setVoltage(anyDouble());
    verify(mockMotor).setVoltage(0.0);
    verify(mockMotor, times(1)).setVoltage(AdditionalMatchers.gt(0.0));

    // Check that telemetry was sent to dashboard
    launcher.periodic();
    readTelemetry();
    assertEquals(fakeCurrent, telemetryDoubleMap.get("Launcher Current"), DELTA);
    assertEquals(fakeVelocity, telemetryDoubleMap.get("Launcher Velocity"), DELTA);
  }

  // ---------- Utility Functions --------------------------------------

  /* Read in telemetry values from the network table and store in maps */
  private int readTelemetry() {
    NetworkTable telemetryTable = NetworkTableInstance.getDefault().getTable("SmartDashboard");
    Set<String> telemetryKeys = telemetryTable.getKeys();

    for (String keyName : telemetryKeys) {
      NetworkTableType entryType = telemetryTable.getEntry(keyName).getType();

      if (entryType == NetworkTableType.kDouble) {
        telemetryDoubleMap.put(keyName, telemetryTable.getEntry(keyName).getDouble(-1));
      } else if (entryType == NetworkTableType.kBoolean) {
        telemetryBooleanMap.put(keyName, telemetryTable.getEntry(keyName).getBoolean(false));
      }
    }
    return telemetryKeys.size();
  }
}
