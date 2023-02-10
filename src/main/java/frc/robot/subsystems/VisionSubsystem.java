package frc.robot.subsystems;

import java.util.List;

import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.subsystems.DriveSubsystem;
import edu.wpi.first.math.MathUtil;

public class VisionSubsystem extends SubsystemBase {

    DriveSubsystem m_driveSubsystem;

    // change nickname later
    PhotonCamera rightCamera = new PhotonCamera("Table");
    PhotonCamera leftCamera = new PhotonCamera("Chair");
    // PhotonCamera rightCamera = new PhotonCamera("Logitech_Webcam_C930e");

    boolean inRotTolerance = false;
    boolean inLatTolerance = false;
    boolean prevTarget = false;

    double xSpeed;
    double leftYSpeed;
    double ySpeed;
    double rotation;
    double distance;
    double calculatedRotation;
    Transform3d rightAprilTransform3d;
    Transform3d leftAprilTransform3d;
    Transform3d targetTransform;
    PhotonTrackedTarget leftTarget;
    PhotonTrackedTarget rightTarget;
    boolean rightTargetValid;
    boolean rightTargetIDValid;
    boolean leftTargetValid;
    boolean leftTargetIDValid;
    boolean anyTargetValid;
    double cameraOffset;

    double aprilTagOffset = -Units.inchesToMeters(28);

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
        for(PhotonTrackedTarget i:getTargets(getResult(camera))){
            double value = i.getBestCameraToTarget().getX();
            if(value < closest){
                closest = value;
                target = i;
            }
        }
        return target;
    }

    public void centerAprilTag(double aprilTagOffset) {

        // right camera stuff
        var rightResult = rightCamera.getLatestResult();
        // checking for targets in view
        if (rightResult.hasTargets()) {
            // checks if a certain target id is seen
            rightTarget = getAprilTagByID(rightResult.getTargets(), getClosestTarget(rightCamera).getFiducialId());

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
        // checking for targets in view
        if (leftResult.hasTargets()) {
            // checks if a certain target id is seen
            leftTarget = getAprilTagByID(leftResult.getTargets(), getClosestTarget(leftCamera).getFiducialId());

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
            double targetAngle = Units.radiansToDegrees(targetTransform.getRotation().getZ());
            double positiveAngle;
            // determining the sign of the angle of the target
            positiveAngle = Math.signum(targetAngle);

            // variables that will be applied to the drive substystem
            xSpeed = 0;
            rotation = ((180 - Math.abs(targetAngle)) * positiveAngle);
            distance = targetTransform.getX() - Constants.RIGHT_CAMERA_OFFSET_BACK;
            calculatedRotation = ((180 - Math.abs(targetAngle)) * positiveAngle);

            // calculation rotation
            if (!inRotTolerance) {
                rotation = MathUtil.clamp(Math.abs(calculatedRotation), 2, 50) * positiveAngle;
                System.out.println("rotation speed: " + rotation);
            } else {
                rotation = 0;
            }

            // rotation deadband
            if (Math.abs(calculatedRotation) > 6) {
                inRotTolerance = false;
            }
            // rotation tolerance
            if (Math.abs(calculatedRotation) < 3) {
                inRotTolerance = true;
            }

            // if rotation is right then correct for lateral
            if (!inLatTolerance && inRotTolerance) {
                double ceiling = 0.5;
                if(aprilTagOffset != 0){
                    ceiling = 0.4;
                }
                ySpeed = Math.signum(targetTransform.getY() + cameraOffset/*-.7239*/)
                        * MathUtil.clamp((Math.abs(targetTransform.getY() + cameraOffset)/*-.7239*/), 0.2, ceiling);
            } else if (inLatTolerance && !inRotTolerance) {
                ySpeed = 0;
            } else {
                ySpeed = 0;
            }

            // lateral deadband
            if (targetTransform.getY() + cameraOffset < -0.1 || targetTransform.getY() + cameraOffset > 0.1) {
                inLatTolerance = false;
            }
            // lateral tolerance
            if (targetTransform.getY() + cameraOffset > -0.05 && targetTransform.getY() + cameraOffset < 0.05) {
                inLatTolerance = true;
            }

            // forward movement
            if (targetTransform.getX() - Constants.RIGHT_CAMERA_OFFSET_BACK <= 0.6) {
                xSpeed = 0;
            } else {
                xSpeed = Math.signum(distance) * MathUtil.clamp(Math.abs(distance), 0.2, 1);
            }

            // applying things to the drive assigned above
            m_driveSubsystem.drive(xSpeed, (ySpeed) * 2 * Constants.VISION_LATERAL_SCALING,
                    rotation * 0.1 * Constants.VISION_ROTATION_SCALING, false);
        }

        else {
            // if we have not target rotate in place
            m_driveSubsystem.drive(0, 0, 0.5, false);
        }
    }

    @Override
    public void periodic() {
        
    }
}
