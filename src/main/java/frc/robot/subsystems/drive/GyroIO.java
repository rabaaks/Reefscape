package frc.robot.subsystems.drive;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.math.geometry.Rotation2d;

public interface GyroIO {
    @AutoLog
    public static class GyroIOInputs {
        public boolean connected = false;
        public double yawPosition = 0.0;;
    }

    public default void updateInputs(GyroIOInputs inputs) {}

    public default void reset() {}
}
