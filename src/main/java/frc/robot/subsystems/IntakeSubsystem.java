// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeConstants;

/**
 * The {@code IntakeIntakeSubsystem} class is a subsystem that controls the speed of a intake using
 * a PID Controller and simple motor feedforward. It uses a CANSparkMax motor and a RelativeEncoder
 * to measure the intake's speed. The class provides methods to run the intake at the specified
 * speed, and stop the motor.
 *
 * <p>The IntakeIntakeSubsystem class provides a constructor where hardware dependencies are passed
 * in to allow access for testing. There is also a method provided to create default hardware when
 * those details are not needed outside of the subsystem.
 *
 * <p>Example Usage:
 *
 * <pre>{@code
 * // Create a new instance of IntakeIntakeSubsystem using specified hardware
 * CANSparkMax motor = new CANSparkMax(1, MotorType.kBrushless);
 * RelativeEncoder encoder = motor.getEncoder();
 * intakeHardware = new IntakeIntakeSubsystem.Hardware(motor, encoder);
 * IntakeIntakeSubsystem intakeSubsystem = new IntakeIntakeSubsystem(intakeHardware);
 *
 * // Create a new instance of IntakeIntakeSubsystem using default hardware
 * IntakeIntakeSubsystem intakeSubsystem = new IntakeIntakeSubsystem(initializeHardware());
 *
 * // Run the intake at a specific speed
 * Command runIntakeCommand = intakeSubsystem.runIntake(500.0);
 * runIntakeCommand.schedule();
 *
 * }
 *
 * Code Analysis:
 * - Main functionalities:
 *   - Control the speed of an intake using a PID Controller
 * - Methods:
 *   - {@code periodic()}: Updates the SmartDashboard with information about the intake's state.
 *   - {@code updateIntakeController()}: Generates the motor command using the PID controller and
 *     feedforward.
 *   - {@code runIntake(double setpoint)}: Returns a Command that runs the intake at the
 *     defined speed.
 *   - {@code setSetPoint(double goal)}: Set the setpoint for the intake..
 *   - {@code atSetpoint()}: Returns whether the intake has reached the set point velocity
 *     within limits.
 *   - {@code enable()}: Enables the PID control of the intake.
 *   - {@code disable()}: Disables the PID control of the intake.
 *   - {@code getIntakeSpeed()}: Returns the intake position for PID control and logging.
 *   - {@code getIntakeVoltageCommand()}: Returns the motor commanded voltage.
 *   - {@code loadPreferences()}: Loads the preferences for tuning the controller.
 *   - {@code close()}: Closes any objects that support it.
 *   - Fields:
 *   - {@code private final CANSparkMax motor}: The motor used to control the intake.
 *   - {@code private final RelativeEncoder encoder}: The encoder used to measure the intake's
 *     position.
 *   - {@code private PIDController intakeController}: The PID controller used to
 *     control the intake's speed.
 *   - {@code private Feedforward feedforward}: The feedforward controller used to
 *     calculate the motor output.
 *   - {@code private double pidOutput}: The output of the PID controller.
 *   - {@code private double newFeedforward}: The calculated feedforward value.
 *   - {@code private boolean intakeEnabled}: A flag indicating whether the intake is enabled.
 *   - {@code private double intakeVoltageCommand}: The motor commanded voltage.
 * </pre>
 */
public class IntakeSubsystem extends SubsystemBase implements AutoCloseable {

  /** Hardware components for the intake subsystem. */
  public static class Hardware {
    PWMSparkMax intakeMotor;

    public Hardware(PWMSparkMax intakeMotor) {
      this.intakeMotor = intakeMotor;
    }
  }

  private final PWMSparkMax intakeMotor;

  /** Create a new IntakeSubsystem thats runs a motor with an openloop voltage command. */
  public IntakeSubsystem(Hardware intakeHardware) {
    this.intakeMotor = intakeHardware.intakeMotor;

    setIntakeVoltageCommand(0);
  }

  /**
   * Create hardware devices for the intake subsystem.
   *
   * @return Hardware object containing all necessary devices for this subsystem
   */
  public static Hardware initializeHardware() {
    PWMSparkMax intakeMotor = new PWMSparkMax(IntakeConstants.INTAKE_MOTOR_PORT);

    return new Hardware(intakeMotor);
  }

  /** Returns a Command that runs the intake at the defined voltage. */
  public Command runIntake(double voltage) {
    return new FunctionalCommand(
        () -> setIntakeVoltageCommand(voltage), () -> {}, interrupted -> {}, () -> false, this);
  }

  /** Returns a Command that stops the intake by setting voltage to 0. */
  public Command stopIntake() {
    return new FunctionalCommand(
        () -> setIntakeVoltageCommand(0), () -> {}, interrupted -> {}, () -> false, this);
  }

  /** Set the intake motor commanded voltage and log the new value. */
  public void setIntakeVoltageCommand(double voltage) {
    intakeMotor.setVoltage(voltage);
    DataLogManager.log("Intake Voltage Set to " + voltage);
    SmartDashboard.putNumber("Intake Voltage", voltage);
  }

  /** Returns the intake motor commanded voltage. */
  public double getIntakeVoltageCommand() {
    return intakeMotor.get();
  }

  /** Disable the intake by setting voltage to 0. */
  public void disableIntake() {
    setIntakeVoltageCommand(0);
  }

  /** Close any objects that support it. */
  @Override
  public void close() {
    intakeMotor.close();
  }
}
