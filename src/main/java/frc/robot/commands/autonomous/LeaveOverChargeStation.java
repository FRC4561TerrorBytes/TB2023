// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autonomous;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.DriveSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class LeaveOverChargeStation extends SequentialCommandGroup {
  /** Creates a new LeaveOverChargeStation. */
  public LeaveOverChargeStation(DriveSubsystem driveSubsystem, double xSpeedInit, double xSpeedUp, double xSpeedFlat, double xSpeedDown, double xSpeedFinal) {
    addCommands(
      new DriveUntilCommand(driveSubsystem, xSpeedInit, 0.0, driveSubsystem::onChargeStation),
      new DriveUntilCommand(driveSubsystem, xSpeedUp, 0.0, driveSubsystem::onPitchDown),
      //TODO clean up a little
      new DriveUntilCommand(driveSubsystem, xSpeedDown, 0.0, driveSubsystem::offChargeStation),
      new DriveUntilCommand(driveSubsystem, xSpeedFlat, 0.0, driveSubsystem::offPitchDown),
      new DriveUntilCommand(driveSubsystem, xSpeedDown, 0.0, driveSubsystem::offChargeStation),
      new DriveUntilCommand(driveSubsystem, xSpeedFinal, 0.0, () -> false).withTimeout(0.5)
    );
  }
}
