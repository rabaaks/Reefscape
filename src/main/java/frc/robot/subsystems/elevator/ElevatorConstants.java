package frc.robot.subsystems.elevator;

import static frc.robot.Constants.*;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;

public class ElevatorConstants {
    public record Config(
        FeedforwardConfig feedforward,
        FeedbackConfig feedback,

        double maxProfileAcceleration,
        double maxProfileVelocity,
        double maxProfileVoltage
    ) {}

    public record IOConfig(
        double radius,
        double gearing,

        double maxHeight,
        double minHeight
    ) {}

    public record SimConfig(
        DCMotor motor,
        double mass
    ) {}

    public record RealConfig(
        int leftMotorId,
        int rightMotorId
    ) {}

    public static final Config protoConfig = new Config(
        new FeedforwardConfig(
            0.23294,
            0.083764,
            9.2729,
            1.2486
        ),
        new FeedbackConfig(
            2.7763,
            0.0,
            0.54642
        ),
        0.5,
        0.5,
        12.0
    );

    public static final IOConfig protoIOConfig = new IOConfig(
        Units.inchesToMeters(0.9125),
        11.25,
        Units.inchesToMeters(44.0),
        Units.inchesToMeters(0.0)
    );

    public static final SimConfig protoSimConfig = new SimConfig(
        DCMotor.getNEO(2),
        Units.lbsToKilograms(30.0)
    );

    public static final RealConfig protoRealConfig = new RealConfig(
        31,
        32
    );

    public static final double sysIdThreshold = 0.1;

    public static final double sysIdRampUp = 2.5;
    public static final double sysIdStep = 5.5;
    public static final double sysIdTimeout = 20.0;
}