// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ArmSubsystem;

public class ResetArmCommand extends CommandBase {
  private ArmSubsystem m_armSubsystem;

  /** Creates a new ResetArmCommand. */
  public ResetArmCommand(ArmSubsystem armSubsystem) {
    m_armSubsystem = armSubsystem;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_armSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_armSubsystem.setArmSpeed(0.1, -0.1);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_armSubsystem.resetPosition();
    m_armSubsystem.setArmSpeed(0.0, 0.0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return m_armSubsystem.armLimitsReached();
  }
}
