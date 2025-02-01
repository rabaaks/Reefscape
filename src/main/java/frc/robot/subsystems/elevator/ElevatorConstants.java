package frc.robot.subsystems.elevator;

import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;

public class ElevatorConstants {
    public static final DCMotor motor = DCMotor.getNEO(2);

    public static final double mass = Units.lbsToKilograms(30.0); // change to actual weight
    public static final double radius = Units.inchesToMeters(0.9175);
    public static final double gearing = 6.75;

    public static final double maxHeight = Units.inchesToMeters(44.0);
    public static final double minHeight = Units.inchesToMeters(0.0);
    public static final double startingHeight = minHeight;

    public static final double p = 5.1253;
    public static final double i = 0.0;
    public static final double d = 1.1194;

    public static final double s = 0.37879;
    public static final double g = 0.80439;
    public static final double v = 6.9315;
    public static final double a = 1.2464;

    public static final double maxProfileVelocity = 2.0;
    public static final double maxProfileAcceleration = 2.0;

    public static final int leftMotorId = 30;
    public static final int rightMotorId = 31;

    public static final double positionConversionFactor = 2.0 * Math.PI * radius / gearing;
    public static final double velocityConversionFactor = positionConversionFactor / 60.0;

    public static final double sysIdMinPosition = 0.1;
    public static final double sysIdMaxPosition = 0.9;

    public static final double sysIdRampUp = 1.5;
    public static final double sysIdStep = 4.5;
    public static final double sysIdTimeout = 20.0;
}