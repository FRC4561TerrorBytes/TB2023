package frc.robot.subsystems;

import java.util.ArrayList;
import java.util.List;

import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.GameState;
import frc.robot.GameState.CenteredState;

public class VisionSubsystem extends SubsystemBase {
    private static final int OUT_OF_ROT_TOLERANCE_DEBOUNCE = 4;
    private static final int LOST_TARGET_DEBOUNCE = 4;

    DriveSubsystem m_driveSubsystem;

    // change nickname later
    PhotonCamera rightCamera = new PhotonCamera("vanap");
    PhotonCamera leftCamera = new PhotonCamera("Table");

    boolean inRotTolerance = false;
    int outOfRotToleranceDebounceCount = OUT_OF_ROT_TOLERANCE_DEBOUNCE;
    boolean inLatTolerance = false;
    int lostTargetDebouceCount = 0;

    double xSpeed;
    double ySpeed;
    double rotation;
    double distance;
    double calculatedRotation;
    Transform3d rightAprilTransform3d;
    Transform3d leftAprilTransform3d;
    Transform3d targetTransform;
    PhotonTrackedTarget leftTarget;
    PhotonTrackedTarget rightTarget;
    boolean rightTargetIDValid;
    boolean leftTargetIDValid;
    double cameraOffset;

    List<Double> averageDistance = new ArrayList<Double>();
    List<Double> averageLateral = new ArrayList<Double>();
    int runningAverageLength = 2;

    public VisionSubsystem(DriveSubsystem driveSubsystem) {
        m_driveSubsystem = driveSubsystem;
    }

    public PhotonPipelineResult getResult(PhotonCamera camera) {
        return camera.getLatestResult();
    }

    public boolean hasTarget(PhotonPipelineResult result) {
        return result.hasTargets();
    }

    public List<PhotonTrackedTarget> getTargets(PhotonPipelineResult result) {
        List<PhotonTrackedTarget> targets = result.getTargets();
        return targets;
    }

    public Transform3d getTransform(PhotonCamera camera) {
        PhotonTrackedTarget target = camera.getLatestResult().getBestTarget();
        return target.getBestCameraToTarget();
    }

    public PhotonTrackedTarget getAprilTagByID(List<PhotonTrackedTarget> targets, int ID) {
        for (PhotonTrackedTarget t : targets) {
            if (t.getFiducialId() == ID) {
                return t;
            }
        }
        return null;
    }

    public PhotonTrackedTarget getClosestTarget(PhotonCamera camera){
        double closest = 100;
        PhotonTrackedTarget target = null;
        List<PhotonTrackedTarget> targetList = getTargets(getResult(camera));
        for(PhotonTrackedTarget i : targetList){
            double value = i.getBestCameraToTarget().getX();
            if(value < closest){
                closest = value;
                target = i;
            }
        }
        return target;
    }

    public double getAverage(List<Double> list){
        double total = 0;
        for(Double element : list){
            total += element;
        }
        return (total/list.size());
    }

    /**
     * Obtain centering command here for consistant command tuning and safety.
     * 
     * @param aprilTagOffset the offset (in meters) of the tag to track.
     * @return a new April tag tracking command.
     */
    public Command centerAprilTagCommand(final double aprilTagOffset) {
        return new CenterAprilTag(aprilTagOffset);
    }

    private void centerAprilTag(final double aprilTagOffset) {
        // right camera stuff
        var rightResult = rightCamera.getLatestResult();
        // checking for targets in view
        var rightClosestTarget = getClosestTarget(rightCamera);
        if (rightResult.hasTargets() && rightClosestTarget != null) {
            // checks if a certain target id is seen
            rightTarget = getAprilTagByID(rightResult.getTargets(), rightClosestTarget.getFiducialId());

            // setting boolean if we see a target and the transform of the target
            if (rightTarget != null) {
                rightTargetIDValid = true;
                rightAprilTransform3d = rightTarget.getBestCameraToTarget();
            } else {
                rightTargetIDValid = false;
                rightAprilTransform3d = null;
            }
        } else {
            rightTargetIDValid = false;
        }

        // left camera stuff
        var leftResult = leftCamera.getLatestResult();
        var leftClosestTarget = getClosestTarget(leftCamera);
        // checking for targets in view
        if (leftResult.hasTargets() && leftClosestTarget != null) {
            // checks if a certain target id is seen
            leftTarget = getAprilTagByID(leftResult.getTargets(), leftClosestTarget.getFiducialId());

            // setting boolean if we see a target and the transform of the target
            if (leftTarget != null) {
                leftTargetIDValid = true;
                leftAprilTransform3d = leftTarget.getBestCameraToTarget();
            } else {
                leftTargetIDValid = false;
                leftAprilTransform3d = null;
            }
        } else {
            leftTargetIDValid = false;
        }

        // if both cameras have a target then it checks pose ambiguity
        if (rightTargetIDValid && leftTargetIDValid) {
            // check pose ambiguity
            double leftPoseAmbiguity = leftTarget.getPoseAmbiguity();
            double rightPoseAmbiguity = rightTarget.getPoseAmbiguity();

            // lower pose ambiguity means more certain
            if (leftPoseAmbiguity >= rightPoseAmbiguity) {
                targetTransform = rightAprilTransform3d;
                cameraOffset = Constants.RIGHT_CAMERA_OFFSET_RIGHT - aprilTagOffset;
            } else if (leftPoseAmbiguity < rightPoseAmbiguity) {
                targetTransform = leftAprilTransform3d;
                cameraOffset = Constants.LEFT_CAMERA_OFFSET_RIGHT - aprilTagOffset;
            }

        }

        // setting variables n stuff
        // if we see a target with the right camera then assign offsets and transform
        // using right camera
        else if (rightTargetIDValid) {
            targetTransform = rightAprilTransform3d;
            cameraOffset = Constants.RIGHT_CAMERA_OFFSET_RIGHT - aprilTagOffset;
        }

        // if we see a target with the left camera then assign offsets and transform
        // using left camera
        else if (leftTargetIDValid) {
            targetTransform = leftAprilTransform3d;
            cameraOffset = Constants.LEFT_CAMERA_OFFSET_RIGHT - aprilTagOffset;
        }

        // settting stuff to null
        else {
            targetTransform = null;
        }

        // checking if a camera sees a target
        if (targetTransform != null) {
            lostTargetDebouceCount = 0;
            averageDistance.add(targetTransform.getX());
            if(averageDistance.size() > runningAverageLength){
                averageDistance.remove(0);
            }

            averageLateral.add(targetTransform.getY() + cameraOffset);
            if(averageLateral.size() > runningAverageLength){
                averageLateral.remove(0);
            }

            final double xAverage = getAverage(averageDistance);
            final double yAverage = getAverage(averageLateral);

            final double targetAngle = Units.radiansToDegrees(targetTransform.getRotation().getZ());
            // determining the sign of the angle of the target
            final double positiveAngle = Math.signum(targetAngle);

            // variables that will be applied to the drive substystem
            xSpeed = 0;
            distance = xAverage - Constants.RIGHT_CAMERA_OFFSET_BACK;
            calculatedRotation = ((180.0 - Math.abs(targetAngle)) * positiveAngle);
            
            // averageRotation.add(calculatedRotation);
            // if(averageRotation.size() > runningAverageLength){
            //     averageRotation.remove(0);
            // }

            // double zAverage = getAverage(averageRotation);

            // calculation rotation
            if (!inRotTolerance) {
                rotation = MathUtil.clamp(Math.abs(calculatedRotation), Constants.VISION_ROTATION_FLOOR_CLAMP, Constants.VISION_ROTATION_CEILING_CLAMP) * positiveAngle;
            } else {
                rotation = 0;
            }

            // forward movement
            if (targetTransform.getX() - Constants.RIGHT_CAMERA_OFFSET_BACK <= Constants.VISION_END_DISTANCE) {
                xSpeed = 0;
            } else {
                xSpeed = Math.signum(distance) * MathUtil.clamp(Math.abs(distance), Constants.VISION_FORWARD_FLOOR_CLAMP, Constants.VISION_FORWARD_CEILING_CLAMP);
            }

            // rotation deadband
            if (Math.abs(calculatedRotation) > Constants.VISION_ROTATION_DEADBAND) {
                outOfRotToleranceDebounceCount++;
                if (outOfRotToleranceDebounceCount >= OUT_OF_ROT_TOLERANCE_DEBOUNCE) {
                    inRotTolerance = false;
                    xSpeed = MathUtil.clamp(distance/2, Constants.VISION_FORWARD_FLOOR_CLAMP, Constants.VISION_FORWARD_CEILING_CLAMP/2);
                }
            }
            // rotation tolerance
            if (Math.abs(calculatedRotation) < Constants.VISION_ROTATION_TOLERANCE) {
                outOfRotToleranceDebounceCount = 0;
                inRotTolerance = true;
            }

            // if rotation is right then correct for lateral
            if (!inLatTolerance && inRotTolerance) {
                ySpeed = Math.signum(yAverage )//+ cameraOffset)
                        * MathUtil.clamp((Math.abs(yAverage)), Constants.VISION_LATERAL_FLOOR_CLAMP, Constants.VISION_LATERAL_CEILING_CLAMP);
            } else if (inLatTolerance && !inRotTolerance) {
                ySpeed = 0;
            } else {
                ySpeed = 0;
            }

            // lateral deadband
            if (yAverage < -Constants.VISION_LATERAL_DEADBAND || yAverage  > Constants.VISION_LATERAL_DEADBAND) {
                inLatTolerance = false;
            }
            // lateral tolerance
            if (yAverage  > -Constants.VISION_LATERAL_TOLERANCE && yAverage  < Constants.VISION_LATERAL_TOLERANCE) {
                inLatTolerance = true;
            }

            // forward movement
            if (distance <= Constants.VISION_END_DISTANCE) {
                xSpeed = 0;
            } else {
                xSpeed = Math.signum(distance) * MathUtil.clamp(Math.abs(distance), Constants.VISION_FORWARD_FLOOR_CLAMP, Constants.VISION_FORWARD_CEILING_CLAMP);
            }

            // applying things to the drive assigned above
            m_driveSubsystem.drive(xSpeed*0.9, (ySpeed) * Constants.VISION_LATERAL_SCALING,
                    -rotation * Constants.VISION_ROTATION_SCALING, false);
            SmartDashboard.putBoolean("In Lat Tol", inLatTolerance);
            SmartDashboard.putBoolean("Rotation Tolerance", inRotTolerance);
            if (inLatTolerance && inRotTolerance) {
                GameState.getInstance().setCenteredState(CenteredState.CENTERED);
            } else if (inRotTolerance && !(inLatTolerance)) {
                GameState.getInstance().setCenteredState(CenteredState.PARTIAL);
            } else {
                GameState.getInstance().setCenteredState(CenteredState.NOTCENTERED);
            }
        }

        else {
            lostTargetDebouceCount++;
        }
    }

    @Override
    public void periodic() {
    }

    private class CenterAprilTag extends CommandBase {
        private final double aprilTagOffset;

        private CenterAprilTag(final double aprilTagOffset) {
            this.aprilTagOffset = aprilTagOffset;
            addRequirements(m_driveSubsystem);
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
            centerAprilTag(aprilTagOffset);
        }

        @Override
        public boolean isFinished() {
            // TODO check for drive stall. That is, up against
            // substation wall or grid edges.
            return lostTargetDebouceCount >= LOST_TARGET_DEBOUNCE;
        }
        
        @Override
        public void end(boolean interrupted) {
            m_driveSubsystem.stop();
        }
    }
}
