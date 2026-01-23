package frc.robot.vision;

import org.photonvision.PhotonCamera;
import org.photonvision.PhotonUtils;
import org.photonvision.targeting.PhotonPipelineResult;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;

import java.util.HashMap;
import java.util.Map;




public class PhotonVisionHelpers {


    public static PhotonPipelineResult getResultOfCamera (PhotonCamera camera) {


        return camera.getLatestResult();
    }

    public static double findDIstance (Pose2d pose1, Pose2d pose2) {


        return PhotonUtils.getDistanceToPose(pose1, pose2);
    }

}
