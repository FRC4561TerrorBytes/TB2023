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
    PhotonCamera aprilTagsCamera = new PhotonCamera("Table");
    //PhotonCamera aprilTagsCamera = new PhotonCamera("Logitech_Webcam_C930e");

    boolean centered = false;
    boolean inRotTolerance = false;
    boolean inLatTolerance = false;



    double xSpeed;
    double ySpeed;
    double rotation;
    double distance;
    double calculatedRotation;
    Transform3d aprilTagTransform;
    boolean targetValid;

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

    public Transform3d getAprilTagTransform(){
        return getTransform(aprilTagsCamera);
    }

    public void centerAprilTag(){
        if(targetValid){
            double targetAngle = Units.radiansToDegrees(aprilTagTransform.getRotation().getZ());

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


            distance = aprilTagTransform.getX() - Constants.CAMERA_OFFSET_BACK;

            
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
                ySpeed = Math.signum(aprilTagTransform.getY())*MathUtil.clamp(Math.abs(aprilTagTransform.getY()), 0.2, 1);
            }
            else{
                //System.out.println("Set 0");
                ySpeed = 0;
            }

            if(Math.abs(aprilTagTransform.getY()) > 0.3){
                //System.out.println("Set false");
                inLatTolerance = false;
            }
            // System.out.println(ySpeed + " " + Math.abs(ySpeed));
            if(Math.abs(aprilTagTransform.getY()) < 0.1){
                //System.out.println("Set true");
                inLatTolerance = true;
            }
            //System.out.println("y: " + getAprilTagTransform().getRotation().getY() + " rotation " + calculatedRotation  + " distance: " + (distance) + " calcY: " + distance*Math.sin(Units.degreesToRadians(calculatedRotation)));
            
            //System.out.println("Y Speed: " + (distance*Math.sin(Units.degreesToRadians(calculatedRotation)) - Constants.CAMERA_OFFSET_RIGHT) + " Actual: " + ySpeed);
            //System.out.println("Y Speed being used: "+ySpeed + " Lat tolerance met: "+inLatTolerance);


            if(inRotTolerance && inLatTolerance){
                centered = true;
            }
            else{
                centered = false;
            }
            
            
            if(aprilTagTransform.getX() - Constants.CAMERA_OFFSET_BACK <= 0.6){
                xSpeed = 0;
            }
            else{
                xSpeed = Math.signum(distance)*MathUtil.clamp(Math.abs(distance), 0.2, 1);
            }
            //System.out.println("X: " + xSpeed + " Y: " + ySpeed + " Rotation: " + rotation);
            //System.out.println("photon vision x: " + getAprilTagTransform().getX() + " distance: " + (distance));
            
            //xSpeed = 0;
            m_driveSubsystem.drive(xSpeed, (ySpeed)*2*Constants.VISION_LATERAL_SCALING, rotation*0.1*Constants.VISION_ROTATION_SCALING, false);
        }
        else{
            m_driveSubsystem.drive(0, 0, 0.5, false);
            System.out.println("No target seen");
        }
    }

    @Override
    public void periodic(){
        targetValid = hasTarget(getResult(aprilTagsCamera));
        if(targetValid){
            aprilTagTransform = getAprilTagTransform(); 
        }
    }
}
