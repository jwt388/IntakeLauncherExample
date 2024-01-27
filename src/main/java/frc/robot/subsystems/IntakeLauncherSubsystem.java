// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeLauncherConstants;
import frc.robot.RobotPreferences;

/**
 * The {@code IntakeLauncherSubsystem} class is a subsystem that controls the speed of a launcher
 * using a PID Controller and simple motor feedforward. It uses a CANSparkMax motor and a 
 * RelativeEncoder to measure the launcher's speed. The class provides methods to run the launcher 
 * at the specified speed, and stop the motor.
 *
 * <p>The IntakeLauncherSubsystem class provides a constructor where hardware dependencies are
 * passed in to allow access for testing. There is also a method provided to create default 
 * hardware when those details are not needed outside of the subsystem.
 *
 * <p>Example Usage:
 *
 * <pre>{@code
 * // Create a new instance of IntakeLauncherSubsystem using specified hardware
 * CANSparkMax motor = new CANSparkMax(1, MotorType.kBrushless);
 * RelativeEncoder encoder = motor.getEncoder();
 * launcherHardware = new IntakeLauncherSubsystem.Hardware(motor, encoder);
 * IntakeLauncherSubsystem launcherSubsystem = new IntakeLauncherSubsystem(launcherHardware);
 *
 * // Create a new instance of IntakeLauncherSubsystem using default hardware
 * IntakeLauncherSubsystem launcherSubsystem = new IntakeLauncherSubsystem(initializeHardware());
 *
 * // Run the launcher at a specific speed
 * Command runLauncherCommand = launcherSubsystem.runLauncher(500.0);
 * runLauncherCommand.schedule();
 *
 * }
 *
 * Code Analysis:
 * - Main functionalities:
 *   - Control the speed of an launcher using a PID Controller
 * - Methods:
 *   - {@code periodic()}: Updates the SmartDashboard with information about the launcher's state.
 *   - {@code updateLauncherController()}: Generates the motor command using the PID controller and 
 *     feedforward.
 *   - {@code runLauncher(double setpoint)}: Returns a Command that runs the launcher at the 
 *     defined speed.
 *   - {@code setSetPoint(double goal)}: Set the setpoint for the launcher..
 *   - {@code atSetpoint()}: Returns whether the launcher has reached the set point velocity 
 *     within limits.
 *   - {@code enable()}: Enables the PID control of the launcher.
 *   - {@code disable()}: Disables the PID control of the launcher.
 *   - {@code getLauncherSpeed()}: Returns the launcher position for PID control and logging.
 *   - {@code getLauncherVoltageCommand()}: Returns the motor commanded voltage.
 *   - {@code loadPreferences()}: Loads the preferences for tuning the controller.
 *   - {@code close()}: Closes any objects that support it.
 *   - Fields:
 *   - {@code private final CANSparkMax motor}: The motor used to control the launcher.
 *   - {@code private final RelativeEncoder encoder}: The encoder used to measure the launcher's
 *     position.
 *   - {@code private PIDController launcherController}: The PID controller used to
 *     control the launcher's speed.
 *   - {@code private Feedforward feedforward}: The feedforward controller used to
 *     calculate the motor output.
 *   - {@code private double pidOutput}: The output of the PID controller.
 *   - {@code private double newFeedforward}: The calculated feedforward value.
 *   - {@code private boolean launcherEnabled}: A flag indicating whether the launcher is enabled.
 *   - {@code private double launcherVoltageCommand}: The motor commanded voltage.
 * </pre>
 */
public class IntakeLauncherSubsystem extends SubsystemBase implements AutoCloseable {

  /** Hardware components for the launcher subsystem. */
  public static class Hardware {
    CANSparkMax launcherMotor;
    RelativeEncoder launcherEncoder;

    public Hardware(CANSparkMax launcherMotor, RelativeEncoder launcherEncoder) {
      this.launcherMotor = launcherMotor;
      this.launcherEncoder = launcherEncoder;
    }
  }

  private final CANSparkMax launcherMotor;
  private final RelativeEncoder launcherEncoder;

  private PIDController launcherController =
      new PIDController(IntakeLauncherConstants.LAUNCHER_KP.getValue(), 0.0, 0.0);

  SimpleMotorFeedforward feedforward =
      new SimpleMotorFeedforward(
          IntakeLauncherConstants.LAUNCHER_KS_VOLTS.getValue(),
          IntakeLauncherConstants.LAUNCHER_KV_VOLTS_PER_RPM.getValue(),
          IntakeLauncherConstants.LAUNCHER_KA_VOLTS_PER_RPM2.getValue());

  private double pidOutput = 0.0;
  private double newFeedforward = 0;
  private boolean launcherEnabled;
  private double launcherVoltageCommand = 0.0;

  /** Create a new LauncherSubsystem controlled by a Profiled PID COntroller . */
  public IntakeLauncherSubsystem(Hardware launcherHardware) {
    this.launcherMotor = launcherHardware.launcherMotor;
    this.launcherEncoder = launcherHardware.launcherEncoder;

    initializeLauncher();
  }

  private void initializeLauncher() {

    RobotPreferences.initPreferencesArray(IntakeLauncherConstants.getLauncherPreferences());

    initEncoder();
    initMotor();

    // Set tolerances that will be used to determine when the launcher is at the goal velocity.
    launcherController.setTolerance(IntakeLauncherConstants.LAUNCHER_TOLERANCE_RPM);

    disable();
  }

  private void initMotor() {
    launcherMotor.restoreFactoryDefaults();
    // Maybe we should print the faults if non-zero before clearing?
    launcherMotor.clearFaults();
    // Configure the motor to use EMF braking when idle and set voltage to 0.
    launcherMotor.setIdleMode(IdleMode.kBrake);
    DataLogManager.log("Launcher motor firmware version:" + launcherMotor.getFirmwareString());
  }

  private void initEncoder() {
    // Setup the encoder scale factors and reset encoder to 0. Since this is a relation encoder,
    // launcher position will only be correct if the launcher is in the starting rest position when
    // the subsystem is constructed.
    launcherEncoder.setPositionConversionFactor(
        IntakeLauncherConstants.LAUNCHER_ROTATIONS_PER_ENCODER_ROTATION);
    launcherEncoder.setVelocityConversionFactor(
        IntakeLauncherConstants.LAUNCHER_ROTATIONS_PER_ENCODER_ROTATION);
  }

  /**
   * Create hardware devices for the launcher subsystem.
   *
   * @return Hardware object containing all necessary devices for this subsystem
   */
  public static Hardware initializeHardware() {
    CANSparkMax launcherMotor =
        new CANSparkMax(IntakeLauncherConstants.LAUNCHER_MOTOR_PORT, MotorType.kBrushless);
    RelativeEncoder launcherEncoder = launcherMotor.getEncoder();

    return new Hardware(launcherMotor, launcherEncoder);
  }

  @Override
  public void periodic() {

    SmartDashboard.putBoolean("Launcher Enabled", launcherEnabled);
    SmartDashboard.putNumber("Launcher Setpoint", launcherController.getSetpoint());
    SmartDashboard.putNumber("Launcher Velocity", launcherEncoder.getVelocity());
    SmartDashboard.putNumber("Launcher Position", launcherEncoder.getPosition());
    SmartDashboard.putNumber("Launcher Voltage", launcherVoltageCommand);
    SmartDashboard.putNumber("Launcher Current", launcherMotor.getOutputCurrent());
    SmartDashboard.putNumber("Launcher Feedforward", newFeedforward);
    SmartDashboard.putNumber("Launcher PID output", pidOutput);
  }

  /** Generate the motor command using the PID controller output and feedforward. */
  public void updateLauncherController() {
    if (launcherEnabled) {
      // Calculate the the motor command by adding the PID controller output and feedforward to run
      // the launcher at the desired speed. Store the individual values for logging.
      pidOutput = launcherController.calculate(getLauncherSpeed());
      newFeedforward = feedforward.calculate(launcherController.getSetpoint());
      launcherVoltageCommand = pidOutput + newFeedforward;

    } else {
      // If the launcher isn't enabled, set the motor command to 0. In this state the launcher
      // will slow down until it stops. Motor EMF braking will cause it to slow down faster
      // if that mode is used.
      pidOutput = 0;
      newFeedforward = 0;
      launcherVoltageCommand = 0;
    }
    launcherMotor.setVoltage(launcherVoltageCommand);
  }

  /** Returns a Command that runs the launcher at the defined speed. */
  public Command runLauncher(double setpoint) {
    return new FunctionalCommand(
        () -> setSetPoint(setpoint),
        this::updateLauncherController,
        interrupted -> disable(),
        () -> false,
        this);
  }

  /**
   * Set the setpoint for the launcher. The PIDController drives the launcher to this speed and
   * holds it there.
   */
  private void setSetPoint(double setpoint) {
    launcherController.setSetpoint(setpoint);

    // Call enable() to configure and start the controller in case it is not already enabled.
    enable();
  }

  /** Returns whether the launcher has reached the set point velocity within limits. */
  public boolean atSetpoint() {
    return launcherController.atSetpoint();
  }

  /**
   * Sets up the PID controller to run the launcher at the defined setpoint velocity. Preferences
   * for tuning the controller are applied.
   */
  private void enable() {

    // Don't enable if already enabled since this may cause control transients
    if (!launcherEnabled) {
      loadPreferences();

      // Reset the PID controller to clear any previous state
      launcherController.reset();
      launcherEnabled = true;

      DataLogManager.log(
          "Launcher Enabled - kP="
              + launcherController.getP()
              + " kI="
              + launcherController.getI()
              + " kD="
              + launcherController.getD()
              + " Setpoint="
              + launcherController.getSetpoint()
              + " CurSpeed="
              + getLauncherSpeed());
    }
  }

  /**
   * Disables the PID control of the launcher. Sets motor output to zero. NOTE: In this state the
   * launcher will slow down until it stops. Motor EMF braking will cause it to slow down faster if
   * that mode is used.
   */
  public void disable() {

    // Clear the enabled flag and update the controller to zero the motor command
    launcherEnabled = false;
    updateLauncherController();

    // Cancel any command that is active
    Command currentCommand = CommandScheduler.getInstance().requiring(this);
    if (currentCommand != null) {
      CommandScheduler.getInstance().cancel(currentCommand);
    }
    DataLogManager.log("Launcher Disabled CurSpeed=" + getLauncherSpeed());
  }

  /** Returns the launcher speed for PID control and logging (Units are RPM). */
  public double getLauncherSpeed() {
    return launcherEncoder.getVelocity();
  }

  /** Returns the launcher motor commanded voltage. */
  public double getLauncherVoltageCommand() {
    return launcherVoltageCommand;
  }

  /**
   * Load Preferences for values that can be tuned at runtime. This should only be called when the
   * controller is disabled - for example from enable().
   */
  private void loadPreferences() {

    // Read Preferences for PID controller
    launcherController.setP(IntakeLauncherConstants.LAUNCHER_KP.getValue());

    // Read Preferences for Feedforward and create a new instance
    double staticGain = IntakeLauncherConstants.LAUNCHER_KS_VOLTS.getValue();
    double velocityGain = IntakeLauncherConstants.LAUNCHER_KV_VOLTS_PER_RPM.getValue();
    double accelerationGain = IntakeLauncherConstants.LAUNCHER_KA_VOLTS_PER_RPM2.getValue();
    feedforward = new SimpleMotorFeedforward(staticGain, velocityGain, accelerationGain);
  }

  /** Close any objects that support it. */
  @Override
  public void close() {
    launcherMotor.close();
  }
}
