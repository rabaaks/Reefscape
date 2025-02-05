package frc.robot.subsystems.vision;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.math.geometry.Pose3d;

public interface CameraIO {
    @AutoLog
    public static class CameraIOInputs {
        public boolean connected = false;
        public Pose3d pose = new Pose3d();
    }

    public default void updateInputs(CameraIOInputs inputs) {}
}
