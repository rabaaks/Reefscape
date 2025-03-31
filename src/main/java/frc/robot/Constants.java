package frc.robot;

import edu.wpi.first.wpilibj.RobotBase;

public class Constants {
    public static final int driverControllerPort = 0;

    public static final Mode simMode = Mode.SIM;
    public static final Mode currentMode = RobotBase.isReal() ? Mode.REAL : simMode;

    public static enum Mode {
        REAL,
        SIM,
        REPLAY
    }

    public record FeedbackConfig(
        double p,
        double i,
        double d
    ) {}

    public record FeedforwardConfig(
        double s,
        double g,
        double v,
        double a
    ) {}
}
