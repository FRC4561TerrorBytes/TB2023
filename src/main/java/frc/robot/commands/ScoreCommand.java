// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.GameState;
import frc.robot.GameState.GamePiece;
import frc.robot.subsystems.IntakeSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class ScoreCommand extends SequentialCommandGroup {
  /** Creates a new ScoreCommand. */
  public ScoreCommand(IntakeSubsystem intakeSubsystem) {
    if(GameState.getInstance().getGamePieceDesired() == GamePiece.CONE){
    addCommands(
      new InstantCommand(() -> GameState.getInstance().setGamePieceHeld(false)),
      new RunCommand(() -> intakeSubsystem.setRollerSpeed(Constants.SCORE_SPEED_CONE), intakeSubsystem)
    );
    } else {
      addCommands(
      new InstantCommand(() -> GameState.getInstance().setGamePieceHeld(false)),
      new RunCommand(() -> intakeSubsystem.setRollerSpeed(Constants.SCORE_SPEED_CUBE), intakeSubsystem)
      );
    };
  }
}
