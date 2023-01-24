package frc.robot.subsystems;

import java.util.List;

import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class VisionSubsystem extends SubsystemBase {
    //change nickname later
    PhotonCamera reflectiveCamera = new PhotonCamera("Table");
    PhotonCamera aprilTagsCamera = new PhotonCamera("Logitech_Webcam_C930e");

    public VisionSubsystem(){

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

    @Override
    public void periodic(){
        //List<PhotonTrackedTarget> reflectiveTargetList = getTargets(getResult(reflectiveCamera));
        //getting yaw is not working work on later
        System.out.println(hasTarget(getResult(aprilTagsCamera)));
    }
}
