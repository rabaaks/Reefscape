package frc.robot.subsystems.vision;

import org.littletonrobotics.junction.AutoLogOutput;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Vision extends SubsystemBase {
    private final Camera[] cameras;

    public Vision(Camera[] cameras) {
        this.cameras = cameras;
    }

    @AutoLogOutput(key="Vision/Poses")
    public Pose3d[] getPoses() {
        Pose3d[] poses = new Pose3d[2];
        for (int i = 0; i < 4; i++) {
            poses[i] = cameras[i].getPose();
        }
        return poses;
    }
}
