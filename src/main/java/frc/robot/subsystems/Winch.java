// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import static frc.robot.Constants.Winch.*;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

public class Winch extends SubsystemBase {

  private final WPI_TalonSRX m_motor = new WPI_TalonSRX(kMotorId);

  /** Creates a new Winch. */
  public Winch() {

    m_motor.configFactoryDefault();
  }

  /**
   * Set motor safety state.
   * @param isEnabled is motor safety enabled
   */
  public void setSafetyEnabled(boolean isEnabled) {
    m_motor.setSafetyEnabled(isEnabled);
  }

  /**
   * Retract the winch.
   */
  public void retract() {
    m_motor.set(kRetractSpeed);
  }

  /**
   * Extend the winch.
   */
  public void extend() {
    m_motor.set(kExtendSpeed);
  }

  /**
   * Stop the winch.
   */
  public void stop() {
    m_motor.set(0.0);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
