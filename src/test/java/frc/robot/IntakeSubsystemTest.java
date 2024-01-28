// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static org.assertj.core.api.Assertions.assertThat;
import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.mockito.Mockito.mock;
import static org.mockito.Mockito.times;
import static org.mockito.Mockito.verify;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.NetworkTableType;
import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.IntakeConstants;
import frc.robot.subsystems.IntakeSubsystem;
import java.util.HashMap;
import java.util.Map;
import java.util.Set;
import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.DisplayName;
import org.junit.jupiter.api.Test;

class IntakeSubsystemTest {
  private static final double DELTA = 5e-3;
  private Map<String, Double> telemetryDoubleMap = new HashMap<>();
  private Map<String, Boolean> telemetryBooleanMap = new HashMap<>();

  private IntakeSubsystem.Hardware intakeHardware;
  private IntakeSubsystem intake;
  private PWMSparkMax mockMotor;

  @BeforeEach
  public void initEach() {
    // Create mock hardware devices
    mockMotor = mock(PWMSparkMax.class);

    // Create subsystem object using mock hardware
    intakeHardware = new IntakeSubsystem.Hardware(mockMotor);
    intake = new IntakeSubsystem(intakeHardware);
  }

  @AfterEach
  public void closeIntake() {
    intake.close(); // motor is closed from the intake close method
  }

  @Test
  @DisplayName("Test constructor and initialization.")
  void testConstructor() {
    // We haven't enabled it yet, so command to motor and saved value should be zero.
    verify(mockMotor).setVoltage(0.0);
    assertThat(intake.getIntakeVoltageCommand()).isZero();
  }

  @Test
  @DisplayName("Test run command and disable.")
  void testMoveCommand() {

    // Create a command to run the intake then initialize
    Command runIntakeCommand = intake.runIntake(IntakeConstants.INTAKE_COMMAND_VOLTS);
    runIntakeCommand.initialize();

    // Verify the voltage command was sent to the motor
    verify(mockMotor, times(1)).setVoltage(IntakeConstants.INTAKE_COMMAND_VOLTS);

    // Verify that telemetry was published
    readTelemetry();
    assertEquals(
        IntakeConstants.INTAKE_COMMAND_VOLTS, telemetryDoubleMap.get("Intake Voltage"), DELTA);

    // Execute the command to run the controller
    runIntakeCommand.execute();
    verify(mockMotor, times(1)).setVoltage(IntakeConstants.INTAKE_COMMAND_VOLTS);

    // When disabled mMotor should be commanded to zero
    intake.disableIntake();
    readTelemetry();
    verify(mockMotor, times(2)).setVoltage(0.0);
    assertThat(telemetryDoubleMap.get("Intake Voltage")).isZero();
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
