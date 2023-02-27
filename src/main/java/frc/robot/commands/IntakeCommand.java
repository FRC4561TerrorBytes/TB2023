// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.GameState;
import frc.robot.GameState.GamePiece;
import frc.robot.subsystems.IntakeSubsystem;

public class IntakeCommand extends CommandBase {
  private IntakeSubsystem m_intakeSubsystem;
  private GamePiece m_gamePieceDesired;
  private Timer timeout = new Timer();

  /** Creates a new IntakeCommand. */
  public IntakeCommand(IntakeSubsystem intakeSubsystem) {
    m_intakeSubsystem = intakeSubsystem;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_intakeSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_gamePieceDesired = GameState.getInstance().getGamePieceDesired();
    timeout.reset();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_intakeSubsystem.intake();
    if (m_intakeSubsystem.isFrontLimitBroken() && m_gamePieceDesired == GamePiece.CUBE) {
      timeout.start();
    }
    if (m_intakeSubsystem.isBackLimitBroken()) {
      timeout.start();
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    GameState.getInstance().setGamePieceHeld(true);
    m_intakeSubsystem.hold();
    timeout.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return timeout.hasElapsed(0.5);
  }
}
