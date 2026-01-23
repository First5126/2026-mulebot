package frc.robot.subsystems;

import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;

import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.constants.CameraConstants;
import frc.robot.vision.PhotonVisionHelpers;

public class PhotonVision implements Subsystem {

    public PhotonCamera camera;

    public PhotonVision() {
        camera = CameraConstants.camera1;
        camera.setFPSLimit(30);
    }

    public void Periodic() {
        PhotonPipelineResult result = PhotonVisionHelpers.getResultOfCamera(camera);
        if (result.hasTargets()) {
            System.out.println(result.getBestTarget());
        }
    }

}

