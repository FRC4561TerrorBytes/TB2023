package frc.robot.subsystems;

import java.util.List;

import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.subsystems.DriveSubsystem;

public class VisionSubsystem extends SubsystemBase {

    DriveSubsystem m_driveSubsystem;

    //change nickname later
    PhotonCamera reflectiveCamera = new PhotonCamera("Table");
    PhotonCamera aprilTagsCamera = new PhotonCamera("Logitech_Webcam_C930e");

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

    public double getYaw(PhotonTrackedTarget target){
        return target.getYaw();
    }

    public Transform3d getTransform(PhotonCamera camera){
        PhotonTrackedTarget target = camera.getLatestResult().getBestTarget();
        return target.getBestCameraToTarget();
    }

    public Transform3d getAprilTagTransform(){
        return getTransform(aprilTagsCamera);
    }

    public void centerAprilTag(){
        m_driveSubsystem.drive(-getAprilTagTransform().getY()*3, 0, 0, false);
        System.out.println(getAprilTagTransform().getY());
    }

    @Override
    public void periodic(){
        //List<PhotonTrackedTarget> reflectiveTargetList = getTargets(getResult(reflectiveCamera));
        //getting yaw is not working work on later
        //System.out.println(hasTarget(getResult(aprilTagsCamera)));
    }
}
