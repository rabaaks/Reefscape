package frc.robot.subsystems.vision;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.math.geometry.Pose3d;

public interface VisionIO {
    @AutoLog
    public static class VisionIOInputs {
        public boolean connected = false;
        
        public Pose3d pose = new Pose3d();
        public double ambiguity = 0.0;
        public int tagCount = 0;
        public double averageTagDistance = 0.0;

        public int[] tagIds = new int[0];
    }

    public default void updateInputs(VisionIOInputs inputs) {}
}
