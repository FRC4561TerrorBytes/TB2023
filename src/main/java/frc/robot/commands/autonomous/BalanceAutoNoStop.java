
// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autonomous;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.DriveSubsystem;
// base OnChargeX -2, OnPitchX -1
public class BalanceAutoNoStop extends SequentialCommandGroup {
  /** Auto that drives forward or backwards until it sees gyro pitch change up and down following balancing on charge station
   * WILL RUN FOREVER IF NOT CALLED WITH TIMEOUT
  */

  public BalanceAutoNoStop(DriveSubsystem driveSubsystem, double onChargeX, double onPitchX) {
    addCommands(
        new DriveUntilCommand(driveSubsystem, onChargeX, 0, driveSubsystem::onChargeStation),
        new DriveUntilCommand(driveSubsystem, onPitchX, 0, driveSubsystem::onPitchDown)
    );
  }
}