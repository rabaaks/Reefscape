package frc.robot.subsystems.drive;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.math.geometry.Rotation2d;

public interface EncoderIO {
    @AutoLog
    public static class EncoderIOInputs {
        public boolean connected = false;
        public double position = 0.0;
    }

    public default void updateInputs(EncoderIOInputs inputs) {}
}
