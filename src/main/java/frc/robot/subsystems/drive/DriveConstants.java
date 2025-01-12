package frc.robot.subsystems.drive;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;

public class DriveConstants {
    public static final DCMotor driveMotor = DCMotor.getNEO(1);
    public static final DCMotor turnMotor = DCMotor.getNEO(1);

    public static final double driveMOI = 0.02;
    public static final double turnMOI = 0.005;

    public static final double driveGearing = 5.36;
    public static final double turnGearing = 150.0 / 7.0;

    public static final double driveP = 0.05;
    public static final double driveI = 0.0;
    public static final double driveD = 0.0;

    public static final double driveS = 0.0;
    public static final double driveV = 0.08;

    public static final double turnP = 8.0;
    public static final double turnI = 0.0;
    public static final double turnD = 0.0;

    public static final double wheelRadius = Units.inchesToMeters(2.0);
    public static final double halfWidth = Units.inchesToMeters(10.0);

    public static final double maxSpeed = 4.0;
}
