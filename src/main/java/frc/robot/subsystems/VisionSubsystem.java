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

public class VisionSubsystem extends SubsystemBase {

    DriveSubsystem m_driveSubsystem;

    //change nickname later
    PhotonCamera aprilTagsCamera = new PhotonCamera("Table");
    //PhotonCamera aprilTagsCamera = new PhotonCamera("Logitech_Webcam_C930e");

    boolean centered = false;
    double xSpeed;
    double ySpeed;
    double rotation;
    double distance;

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
        if(hasTarget(getResult(aprilTagsCamera))){
            double targetAngle = Units.radiansToDegrees(getAprilTagTransform().getRotation().getZ());

            int positiveAngle;
            if(targetAngle < 0){
                positiveAngle = -1;
            }
            else{
                positiveAngle = 1;
            }

            rotation = ((180 - Math.abs(targetAngle))*positiveAngle);

            ySpeed = (getAprilTagTransform().getY() + Constants.CAMERA_OFFSET_RIGHT);
            System.out.println("ySpeed: " + ySpeed);
            xSpeed = 0;

            distance = getAprilTagTransform().getX() - Constants.CAMERA_OFFSET_BACK;

            if(Math.abs(rotation) < (7*distance-4)){
                rotation = 0;
                centered = true;
            }
            else{
                centered = false;
            }

            if(Math.abs(ySpeed) < 0.15){
                ySpeed = 0;
            }
            
            
            if(getAprilTagTransform().getX() - Constants.CAMERA_OFFSET_BACK <= 0.6){
                //m_driveSubsystem.drive(0, ySpeed, 0, false);
                xSpeed = 0;
            }
            else{
                xSpeed = 0.5;
            }
            m_driveSubsystem.drive(xSpeed, ySpeed*Constants.VISION_LATERAL_SCALING, rotation*Constants.VISION_ROTATION_SCALING, false);
            //System.out.println("X: " + xSpeed + " " + "Y: " + ySpeed + " " + "R: " + rotation);
        }
    }

    @Override
    public void periodic(){
        //List<PhotonTrackedTarget> reflectiveTargetList = getTargets(getResult(reflectiveCamera));
        //getting yaw is not working work on later
        //System.out.println(hasTarget(getResult(aprilTagsCamera)));
    }
}
