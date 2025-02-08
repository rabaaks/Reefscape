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
    private final PhotonPoseEstimator photonPoseEstimator;

    private final Transform3d robotToCamera;
    private Pose3d previousPose = new Pose3d();

    public CameraIOReal(String name, Transform3d robotToCamera) {
        camera = new PhotonCamera(name);
        this.robotToCamera = robotToCamera;
        photonPoseEstimator = new PhotonPoseEstimator(aprilTagFieldLayout, PoseStrategy.CLOSEST_TO_REFERENCE_POSE, robotToCamera);
    }

    @Override
    public void updateInputs(CameraIOInputs inputs) {
        inputs.connected = camera.isConnected();

        photonPoseEstimator.setReferencePose(previousPose);
        Pose3d pose = photonPoseEstimator.update(camera.getAllUnreadResults().get(0)).get().estimatedPose;
        inputs.pose = pose;
        previousPose = pose;
    }
}
