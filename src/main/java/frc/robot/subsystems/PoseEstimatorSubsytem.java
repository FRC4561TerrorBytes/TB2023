// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.io.IOException;
import java.util.Optional;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFieldLayout.OriginPosition;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.LimelightHelpers;
import frc.robot.LimelightHelpers.LimelightTarget_Fiducial;

public class PoseEstimatorSubsytem extends SubsystemBase {

  private final AprilTagFieldLayout aprilTagFieldLayout;

  private final SwerveDrivePoseEstimator m_poseEstimator;

  private final Field2d field2d = new Field2d();

  private double previousPipelineTimestamp = 0;

  /** Creates a new PoseEstimatorSubsytem. */
  public PoseEstimatorSubsytem() {
    AprilTagFieldLayout layout;
    try {
      layout = AprilTagFieldLayout.loadFromResource(AprilTagFields.k2023ChargedUp.m_resourceFile);
      var alliance = DriverStation.getAlliance();
      layout.setOrigin(alliance == Alliance.Blue ?
        OriginPosition.kBlueAllianceWallRightSide : OriginPosition.kRedAllianceWallRightSide);
    } catch(IOException err) {
      DriverStation.reportError("Failed to load AprilTagFieldLayout", err.getStackTrace());
      layout = null;
    }

    this.aprilTagFieldLayout = layout;

    m_poseEstimator = new SwerveDrivePoseEstimator(
      Constants.DRIVE_KINEMATICS, 
      DriveSubsystem.getRotation2d(), 
      DriveSubsystem.getModulePositions(),
      new Pose2d());
  }

  public LimelightTarget_Fiducial getClosestTag(String cameraName) {
    double closest = 100;
    LimelightTarget_Fiducial target = null;
    LimelightTarget_Fiducial[] targetList = LimelightHelpers.getLatestResults(cameraName).targetingResults.targets_Fiducials;
    for (LimelightTarget_Fiducial i : targetList) {
      double value = i.tx;
      if (value < closest) {
        closest = value;
        target = i;
      }
    }
    return target;
  }

  @Override
  public void periodic() {
    var limelightResult = LimelightHelpers.getLatestResults("limelight-left");

    var resultTimestamp = limelightResult.targetingResults.timestamp_LIMELIGHT_publish;

    if (resultTimestamp != previousPipelineTimestamp && limelightResult.targetingResults.valid) {
      previousPipelineTimestamp = resultTimestamp;
      var closetTag = getClosestTag("limelight-left");

      Optional<Pose3d> tagPose = aprilTagFieldLayout == null ? Optional.empty() : aprilTagFieldLayout.getTagPose((int)closetTag.fiducialID);

      if (closetTag.fiducialID >= 0 && tagPose.isPresent()) {        
        Pose3d camPose = closetTag.getCameraPose_TargetSpace();

        var visionMeasurement = camPose.transformBy(Constants.CAM_TO_ROBOT);
        m_poseEstimator.addVisionMeasurement(visionMeasurement.toPose2d(), resultTimestamp);
      }

    Logger.getInstance().recordOutput("odometry", getCurrentPose());
    Logger.getInstance().recordOutput("3d pose", new Pose3d(getCurrentPose()));
    }

    m_poseEstimator.update(
      DriveSubsystem.getRotation2d(),
      DriveSubsystem.getModulePositions());


    field2d.setRobotPose(getCurrentPose());
  }

  public Pose2d getCurrentPose() {
    return m_poseEstimator.getEstimatedPosition();
  }

  public void setCurrentPose(Pose2d newPose) {
    m_poseEstimator.resetPosition(
      DriveSubsystem.getRotation2d(),
      DriveSubsystem.getModulePositions(),
      newPose);
  }

  public void resetFieldPosition() {
    setCurrentPose(new Pose2d());
  }
}
