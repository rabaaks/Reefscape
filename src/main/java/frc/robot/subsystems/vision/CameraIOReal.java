package frc.robot.subsystems.vision;

import static frc.robot.subsystems.vision.VisionConstants.*;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform3d;

public class CameraIOReal implements CameraIO {
    private final PhotonCamera camera;

    public CameraIOReal(String name, Transform3d robotToCamera) {
        camera = new PhotonCamera(name);
    }

    @Override
    public void updateInputs(CameraIOInputs inputs) {
        inputs.connected = camera.isConnected();

        inputs.result = camera.getLatestResult();
    }
}
