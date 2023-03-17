// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autonomous;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.ScoreCommand;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.ArmSubsystem.KnownArmPlacement;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.IntakeSubsystem;

public class ScoreCubeLeaveCommunity extends SequentialCommandGroup {
  /**
   * Creates a new ScoreCube.
   * Scores preloaded cube, moves backwards and leaves community immediately
   * after.
   * 
   * @param m_driveSubsyste   The drive subsystem required to drive
   * @param m_armSubsystem    The arm subsystem required to move arm
   * @param m_intakeSubsystem The intake subsystem required to outake / score
   * @param fileName          The string containing the .path file that specifies
   *                          the path to run on each side of the field -
   *                          ("LeaveCommunityRight" or "LeaveCommunityLeft")
   */

  public ScoreCubeLeaveCommunity(DriveSubsystem m_driveSubsystem, ArmSubsystem m_armSubsystem,
      IntakeSubsystem m_intakeSubsystem, double ySpeed) {
    addCommands(
        new InstantCommand(() -> m_armSubsystem.setKnownArmPlacement(KnownArmPlacement.SUBSTATION_APPROACH)),
        new WaitCommand(1.0),
        new InstantCommand(() -> m_armSubsystem.setKnownArmPlacement(KnownArmPlacement.SCORE_CUBE_HIGH)),
        new WaitCommand(1.0),
        new ScoreCommand(m_intakeSubsystem).withTimeout(0.5),
        new DriveUntilCommand(m_driveSubsystem, -0.5, 0, () -> false).withTimeout(0.5),
        new InstantCommand(() -> m_armSubsystem.setKnownArmPlacement(KnownArmPlacement.SUBSTATION_APPROACH)),
        new WaitCommand(1.0),
        new InstantCommand(() -> m_armSubsystem.setKnownArmPlacement(KnownArmPlacement.STOWED)),
        new WaitCommand(1.0),
        new DriveUntilCommand(m_driveSubsystem, -1, ySpeed, () -> false).withTimeout(5)
    );
  }
}
