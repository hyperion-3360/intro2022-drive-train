// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.Winch;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final DriveTrain m_driveTrain = new DriveTrain();
  private final Winch m_winch = new Winch();

  private final XboxController m_controller = new XboxController(0);

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

    // Drive train default is arcade drive
    m_driveTrain.setDefaultCommand(new RunCommand(() -> {
      // Controller y axis is positive right, but z rotation is NWU (positive left)
      double tank_x = m_controller.getRightTriggerAxis() - m_controller.getLeftTriggerAxis();
      double tank_z = -m_controller.getLeftX();
      m_driveTrain.driveArcade(tank_x, tank_z);

      double mecanum_x = -m_controller.getRightY();
      double mecanum_y = -m_controller.getRightX();
      m_driveTrain.driveMecanum(mecanum_x, mecanum_y);

    }, m_driveTrain));

    // Winch default is stop
    m_winch.setDefaultCommand(new RunCommand(m_winch::stop, m_winch));

    // Winch button mapping
    new JoystickButton(m_controller, XboxController.Button.kA.value)
        .whileHeld(new RunCommand(m_winch::retract, m_winch));
    new JoystickButton(m_controller, XboxController.Button.kB.value)
        .whileHeld(new RunCommand(m_winch::extend, m_winch));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // No autonomous for now
    return null;
  }

  /**
   * Set motor safety states for non-drive motors
   * @param isEnabled is the watchdog enabled
   */
  public void setMotorSafetyEnabled(boolean isEnabled) {
    m_driveTrain.setMecanumSafetyEnabled(isEnabled);
    m_winch.setSafetyEnabled(isEnabled);
  }
}
