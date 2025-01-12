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

    public static final double speed = 4.0;
}
