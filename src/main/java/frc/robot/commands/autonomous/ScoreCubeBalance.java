// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autonomous;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.ArmSubsystem.KnownArmPlacement;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.LEDSubsystem;
import frc.robot.subsystems.VisionSubsystem;

public class ScoreCubeBalance extends SequentialCommandGroup {
  /** Creates a new ScoreCubeBalance. */
  private DriveSubsystem m_driveSubsystem;
  private ArmSubsystem m_armSubsystem;
  private VisionSubsystem m_VisionSubsystem;
  private IntakeSubsystem m_intakeSubsystem;
  private LEDSubsystem m_ledSubsystem;
  
  public ScoreCubeBalance() {
    addCommands(
      new RunCommand(() -> m_VisionSubsystem.centerAprilTag(0), m_VisionSubsystem).alongWith(new RunCommand(() -> m_driveSubsystem.updateOdometry(), m_driveSubsystem)),
      new InstantCommand(() -> m_armSubsystem.setKnownArmPlacement(KnownArmPlacement.SCORE_MIDDLE)),
      new WaitCommand(1.0),
      new InstantCommand(() -> m_armSubsystem.setKnownArmPlacement(KnownArmPlacement.SCORE_HIGH)),
      new RunCommand(() -> m_intakeSubsystem.setIntakeSpeed(-Constants.INTAKE_SPEED), m_intakeSubsystem),
      new AutoTrajectory(m_driveSubsystem, "ScoreCubeBalance", 1, 1).getCommandAndStop(),
      new BalanceAuto(m_driveSubsystem)
    );
  }
}
