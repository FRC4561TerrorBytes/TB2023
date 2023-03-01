package frc.robot.commands.autonomous;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.ArmSubsystem.KnownArmPlacement;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.ArmSubsystem.KnownArmPlacement;

public class LeaveCommunity extends SequentialCommandGroup {
  /** Creates a new LeaveCommunity. */

  public LeaveCommunity(DriveSubsystem m_driveSubsystem, ArmSubsystem m_armSubsystem) {
    addCommands(
      new AutoTrajectory(m_driveSubsystem, "LeaveCommunity", 1, 1).getCommandAndStop()
    );
  }
}
