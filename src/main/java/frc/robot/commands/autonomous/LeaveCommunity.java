package frc.robot.commands.autonomous;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.DriveSubsystem;

public class LeaveCommunity extends SequentialCommandGroup {
  /** Drives back and exits community without scoring or balancing
   * @param m_driveSubsyste The drive subsystem required to drive
   * @param m_armSubsystem The arm subsystem required to move arm
   * @param m_intakeSubsystem The intake subsystem required to outake / score
   * @param fileName The string containing the .path file that specifies the path to run on each side of the field - 
   * ("LeaveCommunityRight" or "LeaveCommunityLeft")
  */

  public LeaveCommunity(DriveSubsystem m_driveSubsystem, ArmSubsystem m_armSubsystem, String fileName) {
    addCommands(
      new AutoTrajectory(m_driveSubsystem, fileName, 1, 1).getCommandAndStop()
    );
  }
}
