package frc.robot.subsystems;

import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;

import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.CameraConstants;
import frc.robot.vision.PhotonVisionHelpers;

public class PhotonVision extends SubsystemBase {

    public PhotonCamera camera;

    public PhotonVision() {
        camera = CameraConstants.camera1;
    }

    public void Periodic() {
        System.out.println(camera.isConnected());
        PhotonPipelineResult result = PhotonVisionHelpers.getResultOfCamera(camera);
        if (result.hasTargets()) {
            System.out.println(result.getBestTarget());
        }
    }

}

