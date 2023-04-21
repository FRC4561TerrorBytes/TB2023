
// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autonomous;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.DriveSubsystem;
// base OnChargeX -2, OnPitchX -1
public class BalanceAuto extends SequentialCommandGroup {
  /** Auto that drives forward or backwards until it sees gyro pitch change up and down following balancing on charge station
   * WILL RUN FOREVER IF NOT CALLED WITH TIMEOUT
  */

  public BalanceAuto(DriveSubsystem driveSubsystem, double onChargeX, double onPitchX, double driveSpeed) {
    addCommands(
        new DriveUntilCommand(driveSubsystem, onChargeX, 0, driveSubsystem::onChargeStation),
        new DriveUntilCommand(driveSubsystem, onPitchX, 0, driveSubsystem::onPitchDown),
        new DriveUntilCommand(driveSubsystem, driveSpeed, 0.0, () -> false).withTimeout(0.125)
    );
  }
}