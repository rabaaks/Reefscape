package frc.robot.subsystems.roller;

import org.littletonrobotics.junction.AutoLog;

public interface RollerIO {
    @AutoLog
    public static class ShooterIOInputs {
        public double[] voltages = new double[] {};
        public double[] currents = new double[] {};
    }

    public default void updateInputs(ShooterIOInputs inputs) {}

    public default void setVoltage(double voltage) {}
}
