package frc.robot.subsystems.shooter;

import org.littletonrobotics.junction.AutoLog;

public interface ShooterIO {
    @AutoLog
    public static class ShooterIOInputs {
        public double leftVoltage = 0.0;
        public double rightVoltage = 0.0;

        public double leftCurrent = 0.0;
        public double rightCurrent = 0.0;
    }

    public default void updateInputs(ShooterIOInputs inputs) {}

    public default void setVoltages(double leftVoltage, double rightVoltage) {}
}
