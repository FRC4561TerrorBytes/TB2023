// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autonomous;

import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveSubsystem;

public class DriveUntilCommand extends CommandBase {
  private DriveSubsystem m_driveSubsystem;
  private double m_xSpeed;
  private double m_ySpeed;
  private BooleanSupplier m_endCondition;

  /** Creates a new DriveUntilCommand. */
  public DriveUntilCommand(DriveSubsystem driveSubsystem, double xSpeed, double ySpeed, BooleanSupplier endCondition) {
    m_driveSubsystem = driveSubsystem;
    m_xSpeed = xSpeed;
    m_ySpeed = ySpeed;
    m_endCondition = endCondition;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(driveSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_driveSubsystem.drive(m_xSpeed, m_ySpeed, 0, false);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_driveSubsystem.drive(0, 0, 0, false);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return m_endCondition.getAsBoolean();
  }
}
