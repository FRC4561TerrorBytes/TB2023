// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.LimelightHelpers;
import frc.robot.LimelightHelpers.LimelightResults;

public class VisionSubsytem extends SubsystemBase {

  public VisionSubsytem() {}

  public void updateOdometry() {
    LimelightResults results = LimelightHelpers.getLatestResults("limelight");

    DriveSubsystem.addVision(results.targetingResults.getBotPose2d());
  }

  @Override
  public void periodic() {
    updateOdometry();
  }
}
