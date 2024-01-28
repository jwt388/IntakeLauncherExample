// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.OIConstants;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.LauncherSubsystem;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...

  // First we do things that are in all Robots.
  private PowerDistribution pdp = new PowerDistribution();

  // The driver's controller
  private CommandXboxController operatorController =
      new CommandXboxController(OIConstants.OPERATOR_CONTROLLER_PORT);

  // Now all the subsystems.
  // The Intake and Launcher.
  private final LauncherSubsystem robotLauncher =
      new LauncherSubsystem(LauncherSubsystem.initializeHardware());

  private final IntakeSubsystem robotIntake =
      new IntakeSubsystem(IntakeSubsystem.initializeHardware());

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {

    // Configure the button bindings
    configureButtonBindings();
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    // Run the launcher at the defined speed while the right trigger is held.
    operatorController
        .rightTrigger()
        .whileTrue(
            robotLauncher
                .runLauncher(Constants.LauncherConstants.LAUNCHER_FULL_SPEED)
                .withName("Launcher: Run Full Speed"));

    // Start the intake when the left bumper is pressed.
    operatorController
        .leftBumper()
        .onTrue(
            robotIntake
                .runIntake(Constants.IntakeConstants.INTAKE_COMMAND_VOLTS)
                .withName("Intake: Run"));

    // Stop the intake when the right bumper is pressed.
    operatorController.rightBumper().onTrue(robotIntake.stopIntake().withName("Intake: Stop"));
  }

  /**
   * Disables all subsystems. This should be called on robot disable to prevent integrator windup in
   * subsystems with PID controllers. It also allows subsystems to setup disabled state so
   * simulation matches RoboRio behavior. Commands are canceled at the Robot level.
   */
  public void disableSubsystems() {
    robotIntake.disableIntake();
    robotLauncher.disableLauncher();
    DataLogManager.log("disableSubsystems");
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An ExampleCommand will run in autonomous
    return null;
  }

  /**
   * Use this to get the PDP for data logging.
   *
   * @return The PowerDistribution module.
   */
  public PowerDistribution getPdp() {
    return this.pdp;
  }

  /**
   * Use this to get the Intake Subsystem.
   *
   * @return a reference to the Intake Subsystem
   */
  public IntakeSubsystem getIntakeSubsystem() {
    return robotIntake;
  }

  /**
   * Use this to get the Launcher Subsystem.
   *
   * @return a reference to the Launcher Subsystem
   */
  public LauncherSubsystem getLauncherSubsystem() {
    return robotLauncher;
  }
}
