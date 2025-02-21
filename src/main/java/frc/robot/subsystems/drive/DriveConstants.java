package frc.robot.subsystems.drive;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;

public class DriveConstants {
    public static final double driveMaxSpeed = 4.0;

    public static final double driveSpeed = 3.0;
    public static final double turnSpeed = 4.5;

    public static final DCMotor driveMotor = DCMotor.getNEO(1);
    public static final DCMotor turnMotor = DCMotor.getNEO(1);

    public static final double driveMOI = 0.02;
    public static final double turnMOI = 0.005;

    public static final double driveGearing = 5.14;
    public static final double turnGearing = 12.8;

    public static final double driveP = 3.0;
    public static final double driveI = 0.0;
    public static final double driveD = 0.0;

    public static final double turnP = 12.0;
    public static final double turnI = 0.0;
    public static final double turnD = 0.0;

    public static final double driveS = 0.0;
    public static final double driveV = 2.0;
    public static final double driveA = 0.0;

    public static final double turnS = 0.0;
    public static final double turnV = 0.0;
    public static final double turnA = 0.0;

    public static final double wheelRadius = Units.inchesToMeters(2.0);
    public static final double halfWidth = Units.inchesToMeters(10.0);

    public static final int frontLeftDriveId = 1;
    public static final int frontLeftTurnId =  2;
    public static final int frontLeftEncoderId = 1;
    public static final double frontLeftOffset = 4.3;

    public static final int frontRightDriveId = 3;
    public static final int frontRightTurnId = 4;
    public static final int frontRightEncoderId = 0;
    public static final double frontRightOffset = 4.9;

    public static final int backLeftDriveId = 5;
    public static final int backLeftTurnId = 6;
    public static final int backLeftEncoderId = 2;
    public static final double backLeftOffset = 3.9;

    public static final int backRightDriveId = 7;
    public static final int backRightTurnId = 8;
    public static final int backRightEncoderId = 3;
    public static final double backRightOffset = 3.6;

    public static final int gyroId = 9;
}
