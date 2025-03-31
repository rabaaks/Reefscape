package frc.robot.subsystems.roller;

import org.littletonrobotics.junction.AutoLog;

public interface RollerIO {
    @AutoLog
    public static class RollerIOInputs {
        public double[] voltages = new double[] {};
        public double[] currents = new double[] {};
    }

    public default void updateInputs(RollerIOInputs inputs) {}

    public default void setVoltage(double output) {}
}
