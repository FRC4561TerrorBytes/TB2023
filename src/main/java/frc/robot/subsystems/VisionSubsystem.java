package frc.robot.subsystems;

import java.util.List;

import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class VisionSubsystem extends SubsystemBase {
    //change nickname later
    PhotonCamera camera = new PhotonCamera("Table");

    public VisionSubsystem(){

    }

    public PhotonPipelineResult getResult(){
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
        List<PhotonTrackedTarget> targetList= getTargets(getResult());
        //getting yaw is not working
        System.out.println(hasTarget(getResult()) + getYaw(targetList));
    }
}
