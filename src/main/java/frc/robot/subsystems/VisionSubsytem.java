// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.ArrayList;
import java.util.List;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.GameState;
import frc.robot.GameState.CenteredState;
import frc.robot.LimelightHelpers;
import frc.robot.LimelightHelpers.LimelightResults;
import frc.robot.LimelightHelpers.LimelightTarget_Fiducial;

public class VisionSubsytem extends SubsystemBase {
  private static final int OUT_OF_ROT_TOLERANCE_DEBOUNCE = 4;
  private static final int LOST_TARGET_DEBOUNCE = 20;

  boolean inRotTolerance = false;
  int outOfRotToleranceDebounceCount = OUT_OF_ROT_TOLERANCE_DEBOUNCE;
  boolean inLatTolerance = false;

  DriveSubsystem m_driveSubsytem;

  LimelightTarget_Fiducial leftTag;
  LimelightTarget_Fiducial rightTag;

  Pose3d rightAprilTransform3d;
  Pose3d leftAprilTransform3d;
  Pose3d targetPose;

  boolean rightTargetIDValid;
  boolean leftTargetIDValid;

  double cameraOffset;
  double distance;
  double xSpeed;
  double calcRot;
  double rot;
  double ySpeed;

  int lostTargetDebounceCount;


  List<Double> averageDistance = new ArrayList<Double>();
  List<Double> averageLateral = new ArrayList<Double>();
  int runningAverage = 2;

  public VisionSubsytem(DriveSubsystem driveSubsystem) {
    m_driveSubsytem = driveSubsystem;
  }

  private LimelightTarget_Fiducial getClosestTag(String cameraName) {
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

  public LimelightTarget_Fiducial getTagByID(LimelightTarget_Fiducial[] targets, double ID) {
    for (LimelightTarget_Fiducial t : targets) {
      if (t.fiducialID == ID) {
        return t;
      }
    }
    return null;
  }

  public double getAverage(List<Double> list) {
    double total = 0;
    for (Double element : list) {
      total += element;
    }
    return (total / list.size());
  }

  public Command centerAprilTagCommand(final double aprilTagOffset, final double backOffset) {
    return new CenterAprilTag(aprilTagOffset, backOffset);
  }

  private void centerApriltag(final double aprilTagOffset, final double backOffset) {
    LimelightResults leftResult = LimelightHelpers.getLatestResults("limelight-left");

    var leftClosestTag = getClosestTag("limelight-left");
    if (leftResult.targetingResults.valid && leftClosestTag != null) {
      leftTag = getTagByID(leftResult.targetingResults.targets_Fiducials, leftClosestTag.fiducialID);

      if (leftTag != null) {
        leftTargetIDValid = true;
        leftAprilTransform3d = leftTag.getRobotPose_FieldSpace();
      } else {
        leftTargetIDValid = false;
        leftAprilTransform3d = null;
      }
    } else {
      leftTargetIDValid = false;
    }

    // LimelightResults rightResults = LimelightHelpers.getLatestResults("limelightRight");

    // var rightClosestTag = getClosestTag("limelightRight");
    // if (rightResults.targetingResults.valid && rightClosestTag != null) {
    //   rightTag = getTagByID(rightResults.targetingResults.targets_Fiducials, rightClosestTag.fiducialID);

    //   if (rightTag != null) {
    //     rightTargetIDValid = true;
    //     rightAprilTransform3d = rightTag.getRobotPose_FieldSpace();
    //   } else {
    //     rightTargetIDValid = false;
    //     rightAprilTransform3d = null;
    //   }
    // } else {
    //   rightTargetIDValid = false;
    // }

    // if (rightTargetIDValid) {
    //   targetPose = rightAprilTransform3d;
    //   cameraOffset = Constants.RIGHT_CAMERA_OFFSET_RIGHT - aprilTagOffset;
    // }

    if (leftTargetIDValid) {
      targetPose = leftAprilTransform3d;
      cameraOffset = Constants.LEFT_CAMERA_OFFSET_RIGHT - aprilTagOffset;
    }

    else {
      targetPose = null;
    }

    if (targetPose != null) {
      lostTargetDebounceCount = 0;
      averageDistance.add(targetPose.getX());
      
      if (averageDistance.size() > runningAverage) {
        averageDistance.remove(0);
      }

      averageLateral.add(targetPose.getY() + cameraOffset);
      if (averageLateral.size() > runningAverage) {
        averageLateral.remove(0);
      } 

      final double xAvg = getAverage(averageDistance);
      final double yAvg = getAverage(averageLateral);

      final double targetAngle = Units.radiansToDegrees(targetPose.getRotation().getZ());
      final double posAngle = Math.signum(targetAngle);

      xSpeed = 0;
      distance = xAvg - Constants.RIGHT_CAMERA_OFFSET_BACK;
      calcRot = ((180 - Math.abs(targetAngle)) * posAngle);

      if (!inRotTolerance) {
        rot = MathUtil.clamp(Math.abs(calcRot), Constants.VISION_ROTATION_FLOOR_CLAMP,
          Constants.VISION_ROTATION_CEILING_CLAMP) * posAngle;
      } else {
        rot = 0;
      }

      if (targetPose.getX() - Constants.RIGHT_CAMERA_OFFSET_RIGHT <= backOffset) {
        xSpeed = 0;
      } else {
        xSpeed = Math.signum(distance) * MathUtil.clamp(Math.abs(distance), Constants.VISION_FORWARD_FLOOR_CLAMP,
          Constants.VISION_FORWARD_CEILING_CLAMP);
      }

      if (Math.abs(calcRot) > Constants.VISION_ROTATION_DEADBAND) {
        outOfRotToleranceDebounceCount++;
        if (outOfRotToleranceDebounceCount >= OUT_OF_ROT_TOLERANCE_DEBOUNCE) {
          inRotTolerance = false;
          xSpeed = MathUtil.clamp(distance / 2, Constants.VISION_FORWARD_FLOOR_CLAMP, 
            Constants.VISION_FORWARD_CEILING_CLAMP / 2);
        }
      }

      if (!inLatTolerance && inRotTolerance) {
        ySpeed = Math.signum(yAvg)
          * MathUtil.clamp(Math.abs(yAvg), Constants.VISION_LATERAL_FLOOR_CLAMP, 
            Constants.VISION_LATERAL_CEILING_CLAMP);
      } else {
        ySpeed = 0;
      }

      if (yAvg < -Constants.VISION_LATERAL_DEADBAND || yAvg > Constants.VISION_LATERAL_DEADBAND) {
        inLatTolerance = false;
      }

      if (yAvg > -Constants.VISION_LATERAL_TOLERANCE && yAvg < Constants.VISION_LATERAL_TOLERANCE) {
        inLatTolerance = true;
      }

      if (distance <= backOffset) {
        xSpeed = 0;
      } else {
        xSpeed = Math.signum(distance) * MathUtil.clamp(Math.abs(distance), Constants.VISION_FORWARD_FLOOR_CLAMP,
          Constants.VISION_FORWARD_CEILING_CLAMP);
      }

      m_driveSubsytem.drive(xSpeed * 1.2, (ySpeed) * Constants.VISION_LATERAL_SCALING,
        -rot * Constants.VISION_ROTATION_SCALING, false);
      
      Logger.getInstance().recordOutput("int lat tol", inLatTolerance);
      Logger.getInstance().recordOutput("rot tol", inRotTolerance);

      if (inLatTolerance && inRotTolerance && xSpeed == 0.0) {
        GameState.getInstance().setCenteredState(CenteredState.CENTERED);
      } else if (inRotTolerance && !(inLatTolerance)) {
        GameState.getInstance().setCenteredState(CenteredState.PARTIAL);
      } else {
        GameState.getInstance().setCenteredState(CenteredState.NOTCENTERED);
      }
    }

    else {
      lostTargetDebounceCount++;
    }
  }

  public void updateOdometry() {
    LimelightResults results = LimelightHelpers.getLatestResults("limelight-right");

    // m_driveSubsytem.addVision(results);
  }

  @Override
  public void periodic() {
    updateOdometry();
  }

  private class CenterAprilTag extends CommandBase {
    private final double m_aprilTagOffset;
    private final double m_backOffset;

    private CenterAprilTag(final double aprilTagOffset, final double backOffset) {
      m_aprilTagOffset = aprilTagOffset;
      m_backOffset = backOffset;
      addRequirements(m_driveSubsytem);
    }

    @Override
    public void initialize() {
      inLatTolerance = false;
      inRotTolerance = false;
      outOfRotToleranceDebounceCount = OUT_OF_ROT_TOLERANCE_DEBOUNCE;
      averageDistance.clear();
      averageLateral.clear();
    }

    @Override
    public void execute() {
      centerApriltag(m_aprilTagOffset, m_backOffset);
      //SmartDashboard.putBoolean("driving forward", false);
      //SmartDashboard.putNumber("lost targets", lostTargetDebouceCount);
    }

    @Override
    public boolean isFinished() {
      // TODO check for drive stall. That is, up against
      // substation wall or grid edges.
      if (lostTargetDebounceCount >= LOST_TARGET_DEBOUNCE) {
        return true;
      } else if (GameState.getInstance().getCenteredState() == CenteredState.CENTERED) {
        return true;
      }
      return false;
    }

    @Override
    public void end(boolean interrupted) {
      m_driveSubsytem.stop();
    }
  }
}