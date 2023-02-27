package frc.robot.commands.autonomous;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.DriveSubsystem;

public class LeaveCommunity extends SequentialCommandGroup {
  /** Creates a new LeaveCommunity. */
  private DriveSubsystem m_driveSubsystem = new DriveSubsystem();

  public LeaveCommunity() {
    addCommands(
      new AutoTrajectory(m_driveSubsystem, "LeaveCommunity", 1, 1).getCommandAndStop()
    );
  }
}
