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
import frc.robot.subsystems.VisionSubsystem;
import frc.robot.subsystems.ArmSubsystem.KnownArmPlacement;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class ScoreCube extends SequentialCommandGroup {
  /** Creates a new ScoreCube. */
  private DriveSubsystem m_driveSubsystem = new DriveSubsystem();
  private ArmSubsystem m_armSubsystem = new ArmSubsystem();
  private VisionSubsystem m_visionSubsystem = new VisionSubsystem(m_driveSubsystem);
  private IntakeSubsystem m_intakeSubsystem = new IntakeSubsystem();

  public ScoreCube() {
    addCommands(
      new AutoTrajectory(m_driveSubsystem, "ScoreCubeBalance1", 1, 1).getCommandAndStop(),
      new RunCommand(() -> m_visionSubsystem.centerAprilTag(0), m_visionSubsystem).alongWith(new RunCommand(() -> m_driveSubsystem.updateOdometry(), m_driveSubsystem)),
      new RunCommand(() -> m_armSubsystem.setKnownArmPlacement(KnownArmPlacement.SCORE_HIGH), m_armSubsystem),
      new RunCommand(() -> m_intakeSubsystem.setIntakeSpeed(-Constants.INTAKE_SPEED), m_intakeSubsystem),
      new AutoTrajectory(m_driveSubsystem, "ScoreCube1", 1, 1).getCommandAndStop()
    );
  }
}
