// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autonomous;

import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.LEDSubsystem;
import frc.robot.subsystems.VisionSubsystem;
import frc.robot.subsystems.ArmSubsystem.KnownArmPlacement;

public class ScoreCubeBalance extends SequentialCommandGroup {
  /** Creates a new ScoreCubeBalance. */
  private DriveSubsystem m_driveSubsystem;
  private ArmSubsystem m_armSubsystem;
  private VisionSubsystem m_VisionSubsystem;
  private IntakeSubsystem m_intakeSubsystem;
  private LEDSubsystem m_ledSubsystem;
  
  public ScoreCubeBalance() {
    addCommands(
      new AutoTrajectory(m_driveSubsystem, "ScoreCubeBalance1", 1, 1).getCommandAndStop(),
      new RunCommand(() -> m_VisionSubsystem.centerAprilTag(0), m_VisionSubsystem).alongWith(new RunCommand(() -> m_driveSubsystem.updateOdometry(), m_driveSubsystem)),
      new RunCommand(() -> m_armSubsystem.setKnownArmPlacement(KnownArmPlacement.SCORE_HIGH), m_armSubsystem),
      new RunCommand(() -> m_intakeSubsystem.setIntakeSpeed(-Constants.INTAKE_SPEED), m_intakeSubsystem),
      new AutoTrajectory(m_driveSubsystem, "ScoreCubeBalance2", 1, 1).getCommandAndStop(),
      new BalanceAuto(m_driveSubsystem)
    );
  }
}
