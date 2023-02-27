package frc.robot.commands.autonomous;

import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.ArmSubsystem.KnownArmPlacement;

public class LeaveCommunity extends SequentialCommandGroup {
  /** Creates a new LeaveCommunity. */
  private DriveSubsystem m_driveSubsystem = new DriveSubsystem();
  private ArmSubsystem m_armSubsystem = new ArmSubsystem();

  public LeaveCommunity() {
    addCommands(
      new RunCommand(() -> m_armSubsystem.setKnownArmPlacement(KnownArmPlacement.STOWED), m_armSubsystem),
      new AutoTrajectory(m_driveSubsystem, "LeaveCommunity", 1, 1).getCommandAndStop()
    );
  }
}
