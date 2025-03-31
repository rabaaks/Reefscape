package frc.robot.subsystems.drive;

import static frc.robot.subsystems.drive.DriveConstants.*;

import java.util.ArrayList;
import java.util.List;
import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonPipelineResult;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Transform3d;
import frc.robot.subsystems.drive.DriveConstants.VisionResult;

public class CameraIOReal implements CameraIO {
    private final PhotonCamera camera;
    private final PhotonPoseEstimator poseEstimator;

    public CameraIOReal(CameraIOConfig ioConfig, CameraRealConfig config) {
        camera = new PhotonCamera(config.name());
        poseEstimator = new PhotonPoseEstimator(ioConfig.aprilTagFieldLayout(), PoseStrategy.LOWEST_AMBIGUITY, ioConfig.robotToCamera());
    }

    @Override
    public void updateInputs(CameraIOInputs inputs) {
        inputs.connected = true;

        List<PhotonPipelineResult> results = camera.getAllUnreadResults();
        List<VisionResult> visionResults = new ArrayList<VisionResult>();
        for (PhotonPipelineResult result : results) {
            Optional<EstimatedRobotPose> estimatedPoseOptional = poseEstimator.update(result);
            if (estimatedPoseOptional.isEmpty()) {
                continue;
            }

            EstimatedRobotPose estimatedPose = estimatedPoseOptional.get();

            visionResults.add(
                new VisionResult(
                    estimatedPose.estimatedPose.toPose2d(),
                    estimatedPose.timestampSeconds,
                    VecBuilder.fill(1, 1, 1)
                )
            );
        }

        inputs.visionResults = new VisionResult[visionResults.size()];
        for (int i = 0; i < visionResults.size(); i++) {
            inputs.visionResults[i] = visionResults.get(i);
        }
    }

    @Override
    public void setReferencePose(Pose2d pose) {
        poseEstimator.setReferencePose(pose);
    }
}
