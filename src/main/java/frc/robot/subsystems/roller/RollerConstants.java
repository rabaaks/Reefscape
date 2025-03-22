package frc.robot.subsystems.roller;

public class RollerConstants {
    public record Config(
        double s,
        double v,
        double a
    ) {}

    public record RealConfig(
        int topMotorId,
        int bottomMotorId
    ) {}

    public static final Config protoConfig = new Config(
        0.0,
        1.0,
        0.0
    );

    public static final RealConfig protoRealConfig = new RealConfig(
        21,
        22
    );
}
