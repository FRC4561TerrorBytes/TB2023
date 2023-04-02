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

public class ScoreCube extends SequentialCommandGroup {
  /** 
   * Places cube in autonomous
  */
  public ScoreCube(DriveSubsystem m_driveSubsystem, ArmSubsystem m_armSubsystem, IntakeSubsystem m_intakeSubsystem, KnownArmPlacement placement) {
    addCommands(
      new InstantCommand(() -> m_armSubsystem.setKnownArmPlacement(KnownArmPlacement.SUBSTATION_APPROACH)),
      new WaitCommand(1.0),
      new InstantCommand(() -> m_armSubsystem.setKnownArmPlacement(placement)),
      new WaitCommand(1.0),
      new ScoreCommand(m_intakeSubsystem).withTimeout(0.5),
      new DriveUntilCommand(m_driveSubsystem, -0.5, 0, () -> false).withTimeout(0.5),
      new InstantCommand(() -> m_armSubsystem.setKnownArmPlacement(KnownArmPlacement.SUBSTATION_APPROACH)),
      new WaitCommand(1.0),
      new InstantCommand(() -> m_armSubsystem.setKnownArmPlacement(KnownArmPlacement.STOWED)),
      new WaitCommand(1.0)
    );
  }
}
