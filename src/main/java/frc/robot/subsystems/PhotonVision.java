package frc.robot.subsystems;

import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;

import edu.wpi.first.wpilibj2.command.Subsystem;

public class PhotonVision implements Subsystem {

    public PhotonCamera camera;

    public PhotonVision() {
        camera = new PhotonCamera("MyCameraName");
    }

    public void Periodic() {
    PhotonPipelineResult result = camera.getLatestResult();
    // Use 'result' to get target data, yaw, pitch, etc.
    }

}

