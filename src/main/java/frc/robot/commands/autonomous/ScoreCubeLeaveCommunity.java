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
import frc.robot.subsystems.VisionSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class ScoreCubeLeaveCommunity extends SequentialCommandGroup {
  /** Creates a new ScoreCube. */
  public ScoreCubeLeaveCommunity(DriveSubsystem m_driveSubsystem, ArmSubsystem m_armSubsystem, VisionSubsystem m_visionSubsystem, IntakeSubsystem m_intakeSubsystem) {
    addCommands(
      new InstantCommand(() -> m_armSubsystem.setKnownArmPlacement(KnownArmPlacement.SUBSTATION_APPROACH)),
      new WaitCommand(1.0),
      new InstantCommand(() -> m_armSubsystem.setKnownArmPlacement(KnownArmPlacement.SCORE_CUBE_HIGH)),
      new WaitCommand(1.0),
      new ScoreCommand(m_intakeSubsystem).withTimeout(0.5),
      new DriveUntilCommand(m_driveSubsystem, -0.5, () -> false).withTimeout(0.5),
      new InstantCommand(() -> m_armSubsystem.setKnownArmPlacement(KnownArmPlacement.SUBSTATION_APPROACH)),
      new WaitCommand(1.0),
      new InstantCommand(() -> m_armSubsystem.setKnownArmPlacement(KnownArmPlacement.STOWED)),
      new WaitCommand(1.0),
      new DriveUntilCommand(m_driveSubsystem, -1.0, () -> false).withTimeout(5)
    );
  }
}
