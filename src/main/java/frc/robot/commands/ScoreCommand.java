// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.GameState;
import frc.robot.subsystems.IntakeSubsystem;

public class ScoreCommand extends CommandBase {
  private IntakeSubsystem m_intakeSubsystem;

  /** Creates a new ScoreCommand. */
  public ScoreCommand(IntakeSubsystem intakeSubsystem) {
    m_intakeSubsystem = intakeSubsystem;

    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_intakeSubsystem.score();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    GameState.getInstance().setGamePieceHeld(false);
    m_intakeSubsystem.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
