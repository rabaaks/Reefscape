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

    public static final double driveP = 1.0;
    public static final double driveI = 0.0;
    public static final double driveD = 0.0;

    public static final double driveS = 0.0;
    public static final double driveV = 0.0;
    public static final double driveA = 0.0;

    public static final double turnS = 0.0;
    public static final double turnV = 0.0;
    public static final double turnA = 0.0;

    public static final double turnP = 1.0;
    public static final double turnI = 0.0;
    public static final double turnD = 0.0;

    public static final double wheelRadius = Units.inchesToMeters(2.0);
    public static final double halfWidth = Units.inchesToMeters(10.0);

    public static final double maxSpeed = 4.0;

    public static final double drivePositionConversionFactor = 2.0 * Math.PI * wheelRadius / driveGearing;
    public static final double driveVelocityConversionFactor = drivePositionConversionFactor / 60.0;

    public static final double turnPositionConversionFactor = turnGearing;
    public static final double turnVelocityConversionFactor = turnPositionConversionFactor / 60.0;

    public static final int frontLeftDriveId = 0;
    public static final int frontLeftTurnId =  0;
    public static final int frontRightDriveId = 0;
    public static final int frontRightTurnId = 0;
    public static final int backLeftDriveId = 0;
    public static final int backLeftTurnId = 0;
    public static final int backRightDriveId = 0;
    public static final int backRightTurnId = 0;

}
