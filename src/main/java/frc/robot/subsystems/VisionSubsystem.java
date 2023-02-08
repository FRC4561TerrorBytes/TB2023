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

    //change nickname later
    PhotonCamera rightCamera = new PhotonCamera("Table");
    PhotonCamera leftCamera = new PhotonCamera("Chair");
    //PhotonCamera rightCamera = new PhotonCamera("Logitech_Webcam_C930e");

    boolean centered = false;
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

    public VisionSubsystem(DriveSubsystem driveSubsystem){
        m_driveSubsystem = driveSubsystem;
    }

    public PhotonPipelineResult getResult(PhotonCamera camera){
        return camera.getLatestResult();
    }

    public boolean hasTarget(PhotonPipelineResult result){
        return result.hasTargets();
    }

    public List<PhotonTrackedTarget> getTargets(PhotonPipelineResult result){
        List<PhotonTrackedTarget> targets = result.getTargets();
        return targets;
    }

    public double getYaw(PhotonCamera camera){
        return camera.getLatestResult().getBestTarget().getYaw();
    }

    public Transform3d getTransform(PhotonCamera camera){
        PhotonTrackedTarget target = camera.getLatestResult().getBestTarget();
        return target.getBestCameraToTarget();
    }

    public PhotonTrackedTarget getAprilTagByID(List<PhotonTrackedTarget> targets, int ID){
        for(PhotonTrackedTarget t: targets){
            if(t.getFiducialId() == ID){
                return t;
            }
        }
        return null;
    }

    public Transform3d getrightAprilTransform3d(){
        return getTransform(rightCamera);
    }

    public void centerAprilTag(){
        if(targetTransform != null){
            double targetAngle = Units.radiansToDegrees(targetTransform.getRotation().getZ());

            double positiveAngle;
            positiveAngle = Math.signum(targetAngle);

            xSpeed = 0;
            
            rotation = ((180 - Math.abs(targetAngle))*positiveAngle);


            distance = targetTransform.getX() - Constants.RIGHT_CAMERA_OFFSET_BACK;

            
            calculatedRotation = ((180 - Math.abs(targetAngle))*positiveAngle);


            if(!inRotTolerance){
                rotation = MathUtil.clamp(Math.abs(calculatedRotation), 2, 50)*positiveAngle;
                System.out.println("rotation speed: " + rotation);
            }
            else{
                rotation = 0;
            }

            if(Math.abs(calculatedRotation) > 6){
                inRotTolerance = false;
            }
            if(Math.abs(calculatedRotation) < 3){
                inRotTolerance = true;
            }
            

            if(!inLatTolerance && inRotTolerance){
                System.out.println("in rotation tol, changing ySpeed");
                // ySpeed = Math.signum(targetTransform.getY()+cameraOffset/*-.7239*/)*MathUtil.clamp(Math.abs(targetTransform.getY()+cameraOffset/*-.7239*/), 0.2, 1);
                 ySpeed = Math.signum(targetTransform.getY()+cameraOffset/*-.7239*/)*MathUtil.clamp((Math.abs(targetTransform.getY()+cameraOffset)/*-.7239*/), 0.2, 1);
                //  System.out.println("signum: " + Math.signum(targetTransform.getY()));
                //  System.out.println("abs + offset: " +  (Math.abs(targetTransform.getY())+cameraOffset));
                //  System.out.println("clamp: " + MathUtil.clamp((Math.abs(targetTransform.getY())+cameraOffset/*-.7239*/), 0.2, 1));
                //  System.out.println("yspeed: " + ySpeed);
            }
            else if(inLatTolerance && !inRotTolerance){
                ySpeed = 0;
                System.out.println("in lat tolerance");
            }
            else{
                ySpeed = 0;
                System.out.println("in no tolerance, or all tolerances");
            }
            System.out.println("target and camera offset: " + (targetTransform.getY() + cameraOffset));
            if(targetTransform.getY() + cameraOffset < -0.1 || targetTransform.getY() + cameraOffset > 0.1){
                inLatTolerance = false;
                System.out.println("not in tolerance: " + (targetTransform.getY() + cameraOffset));
            }
            if(targetTransform.getY() + cameraOffset > -0.05 && targetTransform.getY() + cameraOffset < 0.05){
                inLatTolerance = true;
                System.out.println("in tolerance: " + (targetTransform.getY() + cameraOffset));
            }

            

            if(inRotTolerance && inLatTolerance){
                centered = true;
                
            }
            else{
                centered = false;
            }
            
            
            if(targetTransform.getX() - Constants.RIGHT_CAMERA_OFFSET_BACK <= 0.6){
                xSpeed = 0;
            }
            else{
                xSpeed = Math.signum(distance)*MathUtil.clamp(Math.abs(distance), 0.2, 1);
            }

            System.out.println("Raw data Y: " + targetTransform.getY());
            System.out.println("y speed: " + ySpeed);
            m_driveSubsystem.drive(xSpeed, (ySpeed)*2*Constants.VISION_LATERAL_SCALING, rotation*0.1*Constants.VISION_ROTATION_SCALING, false);
        }

        else{
            m_driveSubsystem.drive(0, 0, 0.5, false);
        }
    }

    @Override
    public void periodic(){

        var rightResult = rightCamera.getLatestResult();
        if(rightResult.hasTargets()){
            rightTarget = getAprilTagByID(rightResult.getTargets(), 13);
            
            if(rightTarget != null){
                rightTargetIDValid = true;
                rightAprilTransform3d = rightTarget.getBestCameraToTarget();
            }
            else{
                rightTargetIDValid = false;
                rightAprilTransform3d = null;
            }
        }
        else{
            rightTargetIDValid = false;
        }
        var leftResult = leftCamera.getLatestResult();
        if(leftResult.hasTargets()){
            leftTarget = getAprilTagByID(leftResult.getTargets(), 13);
            
            if(leftTarget != null){
                leftTargetIDValid = true;
                leftAprilTransform3d = leftTarget.getBestCameraToTarget();
            }
            else{
                leftTargetIDValid = false;
                leftAprilTransform3d = null;
            }
        }
        else{
            leftTargetIDValid = false;
        }

        if(rightTargetIDValid && leftTargetIDValid){
            //check pose ambiguity
            double leftPoseAmbiguity = leftTarget.getPoseAmbiguity();
            double rightPoseAmbiguity = rightTarget.getPoseAmbiguity();
            if (leftPoseAmbiguity>=rightPoseAmbiguity)
                {
                    targetTransform = rightAprilTransform3d;
                    cameraOffset = Constants.RIGHT_CAMERA_OFFSET_RIGHT;
                }
            else if (leftPoseAmbiguity<rightPoseAmbiguity)
                {
                    targetTransform = leftAprilTransform3d;
                    cameraOffset = Constants.LEFT_CAMERA_OFFSET_RIGHT;
                }
            
            System.out.println("seeing: both");
        }
        else if(rightTargetIDValid){
            targetTransform = rightAprilTransform3d;
            cameraOffset = Constants.RIGHT_CAMERA_OFFSET_RIGHT;
            // System.out.println("seeing: right");
        }
        else if(leftTargetIDValid){
            targetTransform = leftAprilTransform3d;
            // System.out.println("seeing: left");
            cameraOffset = Constants.LEFT_CAMERA_OFFSET_RIGHT;
        }
        else{
            targetTransform = null;
            // System.out.println("seeing: none");
        }
        
    }
}
