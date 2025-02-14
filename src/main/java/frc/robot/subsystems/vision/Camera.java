package frc.robot.subsystems.vision;

import static frc.robot.subsystems.vision.VisionConstants.aprilTagFieldLayout;

import org.littletonrobotics.junction.Logger;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform3d;
import frc.robot.subsystems.vision.CameraIO.CameraIOInputs;

public class Camera {
    private final CameraIO io;
    private final CameraIOInputsAutoLogged inputs = new CameraIOInputsAutoLogged();

    private final int index;

    private final Transform3d robotToCamera;
    private final PhotonPoseEstimator photonPoseEstimator;
    private Pose3d pose = new Pose3d();

    public Camera(CameraIO io, int index, Transform3d robotToCamera) {
        this.io = io;
        this.index = index;
        this.robotToCamera = robotToCamera;
        photonPoseEstimator = new PhotonPoseEstimator(aprilTagFieldLayout, PoseStrategy.CLOSEST_TO_REFERENCE_POSE, robotToCamera);
    }

    public void updateInputs() {
        io.updateInputs(inputs);
        Logger.processInputs("Vision/Cameras/" + index, inputs);

        photonPoseEstimator.setReferencePose(pose);
        pose = photonPoseEstimator.update(inputs.result).get().estimatedPose;
    }

    public Pose3d getPose() {
        return pose;
    }
}
