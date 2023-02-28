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
import frc.robot.subsystems.VisionSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class ScoreCubeLeaveCommunity extends SequentialCommandGroup {
  /** Creates a new ScoreCube. */
  private DriveSubsystem m_driveSubsystem = new DriveSubsystem();
  private ArmSubsystem m_armSubsystem = new ArmSubsystem();
  private VisionSubsystem m_visionSubsystem = new VisionSubsystem(m_driveSubsystem);
  private IntakeSubsystem m_intakeSubsystem = new IntakeSubsystem();

  public ScoreCubeLeaveCommunity() {
    addCommands(
      new RunCommand(() -> m_visionSubsystem.centerAprilTag(0), m_visionSubsystem).alongWith(new RunCommand(() -> m_driveSubsystem.updateOdometry(), m_driveSubsystem)),
      new InstantCommand(() -> m_armSubsystem.setKnownArmPlacement(KnownArmPlacement.SCORE_MIDDLE)),
      new WaitCommand(1.0),
      new InstantCommand(() -> m_armSubsystem.setKnownArmPlacement(KnownArmPlacement.SCORE_HIGH)),
      new RunCommand(() -> m_intakeSubsystem.setIntakeSpeed(-Constants.INTAKE_SPEED), m_intakeSubsystem),
      new AutoTrajectory(m_driveSubsystem, "LeaveCommunity", 1, 1).getCommandAndStop()
    );
  }
}
