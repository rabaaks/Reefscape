package frc.robot.subsystems.elevator;

import static frc.robot.subsystems.elevator.ElevatorConstants.*;

import org.littletonrobotics.junction.AutoLog;

import frc.robot.Constants.FeedbackConfig;

public interface ElevatorIO {
    @AutoLog
    public static class ElevatorIOInputs {
        public double position = 0.0;
        public double velocity = 0.0;

        public double[] voltages = new double[] {};
        public double[] currents = new double[] {};
    }

    public default void updateInputs(ElevatorIOInputs inputs) {}

    public default void setPosition(double position, double ffVoltage) {}

    public default void reset() {}

    public default void setFeedback(FeedbackConfig feedbackConfig) {}
}
