package frc.robot.commands.autonomous;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.DriveSubsystem;

public class LeaveCommunity extends SequentialCommandGroup {
  /** Creates a new LeaveCommunity. */

  public LeaveCommunity(DriveSubsystem m_driveSubsystem, ArmSubsystem m_armSubsystem) {
    addCommands(
      new DriveUntilCommand(m_driveSubsystem, -2, () -> false).withTimeout(3)
    );
  }
}