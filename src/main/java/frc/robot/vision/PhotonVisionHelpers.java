package frc.robot.vision;

import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;
import java.util.HashMap;
import java.util.Map;




public class PhotonVisionHelpers {

    public static Map<String, PhotonCamera> camerasMap = Map.of(
            "Camera1", new PhotonCamera("Camera1")
        );

    public static PhotonPipelineResult getResultOfCamera (PhotonCamera camera) {


        return camera.getLatestResult();
    }

}
