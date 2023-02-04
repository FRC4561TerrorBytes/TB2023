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
    PhotonCamera leftCamera = new PhotonCamera("Arducam_OV9281_USB_Camera_Num2");
    //PhotonCamera rightCamera = new PhotonCamera("Logitech_Webcam_C930e");

    boolean centered = false;
    boolean inRotTolerance = false;
    boolean inLatTolerance = false;
    boolean prevTarget = false;



    double xSpeed;
    double ySpeed;
    double rotation;
    double distance;
    double calculatedRotation;
    Transform3d rightAprilTransform3d;
    Transform3d leftAprilTransform3d;
    boolean rightTargetValid;
    boolean rightTargetIDValid;
    boolean leftTargetValid;
    boolean leftTargetIDValid;

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

    public Transform3d getAprilTagByID(List<PhotonTrackedTarget> targets, int ID){
        for(PhotonTrackedTarget t: targets){
            if(t.getFiducialId() == ID){
                return t.getBestCameraToTarget();
            }
        }
        return null;
    }

    public Transform3d getrightAprilTransform3d(){
        return getTransform(rightCamera);
    }

    public void centerAprilTag(){
        if(rightTargetIDValid){
            double targetAngle = Units.radiansToDegrees(rightAprilTransform3d.getRotation().getZ());

            double positiveAngle;
            // if(targetAngle < 0){
            //     positiveAngle = -1;
            // }
            // else{
            //     positiveAngle = 1;
            // }
            positiveAngle = Math.signum(targetAngle);

            xSpeed = 0;
            
            rotation = ((180 - Math.abs(targetAngle))*positiveAngle);


            distance = rightAprilTransform3d.getX() - Constants.CAMERA_OFFSET_BACK;

            
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
                //System.out.println("Set speed");
                //ySpeed = distance*Math.sin(Units.degreesToRadians(calculatedRotation)) - Constants.CAMERA_OFFSET_RIGHT;
                ySpeed = Math.signum(rightAprilTransform3d.getY())*MathUtil.clamp(Math.abs(rightAprilTransform3d.getY()), 0.2, 1);
            }
            else{
                //System.out.println("Set 0");
                ySpeed = 0;
            }

            if(Math.abs(rightAprilTransform3d.getY()) > 0.3){
                //System.out.println("Set false");
                inLatTolerance = false;
            }
            // System.out.println(ySpeed + " " + Math.abs(ySpeed));
            if(Math.abs(rightAprilTransform3d.getY()) < 0.1){
                //System.out.println("Set true");
                inLatTolerance = true;
            }
            //System.out.println("y: " + getrightAprilTransform3d().getRotation().getY() + " rotation " + calculatedRotation  + " distance: " + (distance) + " calcY: " + distance*Math.sin(Units.degreesToRadians(calculatedRotation)));
            
            //System.out.println("Y Speed: " + (distance*Math.sin(Units.degreesToRadians(calculatedRotation)) - Constants.CAMERA_OFFSET_RIGHT) + " Actual: " + ySpeed);
            //System.out.println("Y Speed being used: "+ySpeed + " Lat tolerance met: "+inLatTolerance);


            if(inRotTolerance && inLatTolerance){
                centered = true;
            }
            else{
                centered = false;
            }
            
            
            if(rightAprilTransform3d.getX() - Constants.CAMERA_OFFSET_BACK <= 0.6){
                xSpeed = 0;
            }
            else{
                xSpeed = Math.signum(distance)*MathUtil.clamp(Math.abs(distance), 0.2, 1);
            }
            //System.out.println("X: " + xSpeed + " Y: " + ySpeed + " Rotation: " + rotation);
            //System.out.println("photon vision x: " + getrightAprilTransform3d().getX() + " distance: " + (distance));
            
            //xSpeed = 0;
            m_driveSubsystem.drive(xSpeed, (ySpeed)*2*Constants.VISION_LATERAL_SCALING, rotation*0.1*Constants.VISION_ROTATION_SCALING, false);
        }
        else{
            m_driveSubsystem.drive(0, 0, 0.5, false);
        }
    }

    @Override
    public void periodic(){
        // rightTargetValid = hasTarget(getResult(rightCamera));
        // leftTargetValid = hasTarget(getResult(leftCamera));
        // if(rightTargetValid){
        //     rightAprilTransform3d = getTransform(rightCamera); 
        // }
        // if(leftTargetValid){
        //     lefttAprilTransform3d = getTransform(leftCamera);
        // }
        
        // getAprilTagByID(null, 0)

        var rightResult = rightCamera.getLatestResult();
        if(rightResult.hasTargets()){
            rightAprilTransform3d = getAprilTagByID(rightResult.getTargets(), 0);
            if(rightAprilTransform3d != null){
                rightTargetIDValid = true;
            }else{
                rightTargetIDValid = false;
            }
        }
        // var leftResult = leftCamera.getLatestResult();
        // if(leftResult.hasTargets()){
        //     rightAprilTransform3d = getAprilTagByID(leftResult.getTargets(), 0);
        //     if(leftAprilTransform3d != null){
        //         leftTargetIDValid = true;
        //     }else{
        //         leftTargetIDValid = false;
        //     }
        // }

       
        // prevTarget = rightTargetValid;
        
    }
}
