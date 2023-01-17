// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static frc.robot.Constants.Intake.*;

public class Intake extends SubsystemBase {

  private final CANSparkMax m_intake = new CANSparkMax(kIntakeId, MotorType.kBrushless);
  private final TalonSRX m_wheel = new TalonSRX(kWheelId);

  /** Creates a new Intake. */
  public Intake() {
    m_intake.restoreFactoryDefaults();
    m_intake.setInverted(kIntakeInverted);
    m_intake.burnFlash();
    m_wheel.configFactoryDefault();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public Command startIntake() {
    return this.runOnce(() -> m_intake.set(kIntakeSpeed));
  }

  public Command stopIntake() {
    return this.runOnce(() -> m_intake.set(0.0));
  }

  public Command startWheel() {
    return this.runOnce(() -> m_wheel.set(ControlMode.PercentOutput, kWheelSpeed));
  }

  public Command stopWheel() {
    return this.runOnce(() -> m_wheel.set(ControlMode.PercentOutput, 0.0));
  }
}
