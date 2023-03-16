package frc.robot.commands.autonomous;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.DriveSubsystem;

public class LeaveCommunity extends SequentialCommandGroup {
  /**
   * Drives back and exits community without scoring or balancing
   */
  public LeaveCommunity(DriveSubsystem m_driveSubsystem, ArmSubsystem m_armSubsystem, double yDirection) {
    addCommands(
      new DriveUntilCommand(m_driveSubsystem, -1, yDirection, () -> false).withTimeout(5)
    );
  }
}
