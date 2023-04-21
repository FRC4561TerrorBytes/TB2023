// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autonomous;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.DriveSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class ExitChargeStation extends SequentialCommandGroup {
  /** Creates a new ExitChargeStation. */
  public ExitChargeStation(DriveSubsystem m_driveSubsystem) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new DriveUntilCommand(m_driveSubsystem, -1.0, 0, m_driveSubsystem::offPitchDown),
      new DriveUntilCommand(m_driveSubsystem, -1.0, 0, m_driveSubsystem::onFlat)
    );
  }
}
