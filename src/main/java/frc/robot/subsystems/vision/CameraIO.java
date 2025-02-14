package frc.robot.subsystems.vision;

import org.littletonrobotics.junction.AutoLog;
import org.photonvision.targeting.PhotonPipelineResult;

import edu.wpi.first.math.geometry.Pose3d;

public interface CameraIO {
    @AutoLog
    public static class CameraIOInputs {
        public boolean connected = false;
        public PhotonPipelineResult result;
    }

    public default void updateInputs(CameraIOInputs inputs) {}
}
