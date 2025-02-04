package frc.robot.subsystems.vision;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;

public class VisionConstants {
    public static final Transform3d robotPosition = new Transform3d(0.0, 0.0, 0.0, new Rotation3d(0.0, 0.0, 0.0));

    public static final AprilTagFieldLayout aprilTagLayout = AprilTagFieldLayout.loadField(AprilTagFields.k2025Reefscape);
}
