package frc.robot.subsystems.shooter;

import org.littletonrobotics.junction.AutoLog;

public interface ShooterIO {
    @AutoLog
    public static class ShooterIOInputs {
        public double[] voltages = new double[] {};
        public double[] currents = new double[] {};
    }

    public default void updateInputs(ShooterIOInputs inputs) {}

    public default void setVoltage(double voltage) {}
}
