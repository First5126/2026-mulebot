package frc.robot.subsystems;

import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;

import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.vision.PhotonVisionHelpers;

public class PhotonVision implements Subsystem {

    public PhotonCamera camera;

    public PhotonVision() {
        camera = PhotonVisionHelpers.camerasMap.get("Camera1");
    }

    public void Periodic() {
        PhotonPipelineResult result = PhotonVisionHelpers.getResultOfCamera(camera);
        System.out.println(result.getBestTarget());
    }

}

