package frc.robot.subsystems.shooter;

public class ShooterConstants {
    public static final double gearing = 1.0;

    public static final double wheelRadius = 2.0;

    public static final double positionConversionFactor = 2.0 * Math.PI * wheelRadius / gearing;
    public static final double velocityConversionFactor = positionConversionFactor / 60.0;

    public static final double s = 0.0;
    public static final double v = 0.0;
    public static final double a = 0.0;

    public static final double p = 0.0;
    public static final double i = 0.0;
    public static final double d = 0.0;
}
