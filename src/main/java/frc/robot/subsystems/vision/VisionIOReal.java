package frc.robot.subsystems.vision;

import static frc.robot.subsystems.vision.VisionConstants.*;

import org.photonvision.PhotonCamera;
import org.photonvision.targeting.MultiTargetPNPResult;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform3d;

import java.util.ArrayList;
import java.util.List;
import java.util.Optional;
import java.util.stream.Collector;
import java.util.stream.Collectors;

public class VisionIOReal implements VisionIO {
    private final PhotonCamera camera;

    public VisionIOReal(String name) {
        camera = new PhotonCamera(name);
    }

    @Override
    public void updateInputs(VisionIOInputs inputs) {
        inputs.connected = camera.isConnected();
        
        List<Integer> tagIds = new ArrayList<>();
        List<PoseObservation> poseObservations = new ArrayList<>();

        for (PhotonPipelineResult result : camera.getAllUnreadResults()) {
            if (result.multitagResult.isPresent()) {
                MultiTargetPNPResult multitagResult = result.multitagResult.get();

                Transform3d fieldToCamera = multitagResult.estimatedPose.best;
                Transform3d fieldToRobot = fieldToCamera.plus(robotPosition);
                Pose3d robotPose = new Pose3d(fieldToRobot.getTranslation(), fieldToRobot.getRotation());

                double totalTagDistance = 0.0;
                for (PhotonTrackedTarget target : result.targets) {
                    totalTagDistance += target.bestCameraToTarget.getTranslation().getNorm();
                }

                tagIds.addAll(multitagResult.fiducialIDsUsed.stream().map(Short::intValue).collect(Collectors.toList()));

                poseObservations.add(
                    new PoseObservation(
                        result.getTimestampSeconds(),
                        robotPose,
                        multitagResult.estimatedPose.ambiguity,
                        multitagResult.fiducialIDsUsed.size(),
                        totalTagDistance / result.targets.size()
                    )
                );
            } else if (!result.targets.isEmpty()) {
                PhotonTrackedTarget target = result.targets.get(0);

                Optional<Pose3d> tagPose = aprilTagLayout.getTagPose(target.fiducialId);
                if (tagPose.isPresent()) {
                    Transform3d fieldToTarget = new Transform3d(tagPose.get().getTranslation(), tagPose.get().getRotation());
                    Transform3d cameraToTarget = target.bestCameraToTarget;
                    Transform3d fieldToCamera = fieldToTarget.plus(cameraToTarget.inverse());
                    Transform3d fieldToRobot = fieldToCamera.plus(robotPosition.inverse());
                    Pose3d robotPose = new Pose3d(fieldToRobot.getTranslation(), fieldToRobot.getRotation());

                    tagIds.add(target.fiducialId);

                    poseObservations.add(
                        new PoseObservation(
                            result.getTimestampSeconds(),
                            robotPose,
                            target.poseAmbiguity,
                            1,
                            cameraToTarget.getTranslation().getNorm()
                        )
                    );
                }
            }
        }

        inputs.poseObservations = poseObservations.toArray(new PoseObservation[0]);
        inputs.tagIds = tagIds.stream().mapToInt(Integer::intValue).toArray();
    }
}
