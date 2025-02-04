package frc.robot.subsystems.vision;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.math.geometry.Pose3d;

public interface VisionIO {
    public static record PoseObservation(
        double timestamp,
        Pose3d pose,
        double ambiguity,
        int tagCount,
        double averageTagDistance
    ) {};

    @AutoLog
    public static class VisionIOInputs {
        public boolean connected = false;
        
        public PoseObservation[] poseObservations = new PoseObservation[0];

        public int[] tagIds = new int[0];
    }

    public default void updateInputs(VisionIOInputs inputs) {}
}
