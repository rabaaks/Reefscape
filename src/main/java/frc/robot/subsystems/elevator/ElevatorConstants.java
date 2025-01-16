package frc.robot.subsystems.elevator;

import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;

public class ElevatorConstants {
    public static final DCMotor motor = DCMotor.getNEO(2);

    public static final double mass = Units.lbsToKilograms(30); // change to actual weight
    public static final double radius = Units.inchesToMeters(0.918);
    public static final double gearing = 6.75;

    public static final double maxHeight = Units.inchesToMeters(56);
    public static final double minHeight = Units.inchesToMeters(0);
    public static final double startingHeight = minHeight;

    public static final double p = 0.0;
    public static final double i = 0.0;
    public static final double d = 0.0;

    // constants tested with 30 lbs sim
    public static final double s = 0.0;
    public static final double g = 1.063;
    public static final double v = 6.08;
    public static final double a = 0.23;

    public static final double maxProfileVelocity = 6.0;
    public static final double maxProfileAcceleration = 4.0;

    public static final int leftMotorId = 0;
    public static final int rightMotorId = 0;

    public static final double positionConversionFactor = gearing * 2.0 * Math.PI * radius;
    public static final double velocityConversionFactor = positionConversionFactor / 60.0;

    public static final double sysIdMinPosition = 0.2;
    public static final double sysIdMaxPosition = 1.2;
}
