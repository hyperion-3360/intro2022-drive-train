// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveTrain;
import static frc.robot.Constants.DriveTrain.*;

public class LockHeading extends CommandBase {

  private final DriveTrain m_driveTrain;
  private final PIDController m_pid = new PIDController(kP, kI, kD);
  private final NetworkTableEntry m_errorEntry = Shuffleboard.getTab("Vitals").add("Angle Error", 0.0).getEntry();
  private final NetworkTableEntry m_zCommandEntry = Shuffleboard.getTab("Vitals").add("Commanded Z", 0.0).getEntry();

  /** Creates a new LockHeading. */
  public LockHeading(DriveTrain driveTrain) {

    m_driveTrain = driveTrain;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_driveTrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_pid.setSetpoint(m_driveTrain.getRotation().getRadians());
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double zCommand = m_pid.calculate(m_driveTrain.getRotation().getRadians());
    m_driveTrain.driveArcade(0.0, zCommand, false);
    m_errorEntry.setDouble(m_pid.getPositionError());
    m_zCommandEntry.setDouble(zCommand);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_driveTrain.driveArcade(0.0, 0.0, false);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
