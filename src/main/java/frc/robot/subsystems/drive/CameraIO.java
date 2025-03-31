package frc.robot.subsystems.drive;

import static frc.robot.subsystems.drive.DriveConstants.*;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.math.geometry.Pose2d;

public interface CameraIO {
    @AutoLog
    public static class CameraIOInputs {
        public boolean connected = false;
        public VisionResult[] visionResults = new VisionResult[] {};
    }

    public default void updateInputs(CameraIOInputs inputs) {}

    public default void setReferencePose(Pose2d pose) {}
}
