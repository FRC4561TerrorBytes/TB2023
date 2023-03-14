
// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autonomous;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.DriveSubsystem;
// base OnChargeX -2, OnPitchX -1
public class BalanceAuto extends SequentialCommandGroup {
  /** Creates a new BalanceAuto. */

  public BalanceAuto(DriveSubsystem driveSubsystem, double onChargeX, double onPitchX) {
    addCommands(
        new DriveUntilCommand(driveSubsystem, onChargeX, 0, driveSubsystem::onChargeStation),
        new WaitCommand(0.5),
        new DriveUntilCommand(driveSubsystem, onPitchX, 0, driveSubsystem::onPitchDown));
  }
}