package frc.robot.subsystems.elevator;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;

public class ElevatorConstants {
    public static final DCMotor motor = DCMotor.getNEO(2);

    public static final double mass = 8.956;
    public static final double radius = 0.918;
    public static final double gearing = 2.25;

    public static final double maxHeight = Units.inchesToMeters(60);
    public static final double minHeight = Units.inchesToMeters(35);
    public static final double startingHeight = minHeight;

    public static final double p = 0.0;
    public static final double i = 0.0;
    public static final double d = 0.0;
}
