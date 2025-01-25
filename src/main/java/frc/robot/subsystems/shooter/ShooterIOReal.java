package frc.robot.subsystems.shooter;

import static frc.robot.subsystems.shooter.ShooterConstants.*;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.SparkLowLevel.MotorType;

public class ShooterIOReal implements ShooterIO {
    private final SparkMax leftMotor;
    private final SparkMax rightMotor;

    private final RelativeEncoder leftEncoder;
    private final RelativeEncoder rightEncoder;

    private final SparkClosedLoopController leftFeedback;
    private final SparkClosedLoopController rightFeedback;

    public ShooterIOReal(int leftId, int rightId) {
        leftMotor = new SparkMax(leftId, MotorType.kBrushed);
        rightMotor = new SparkMax(rightId, MotorType.kBrushed);

        leftEncoder = leftMotor.getEncoder();
        rightEncoder = rightMotor.getEncoder();

        leftFeedback = leftMotor.getClosedLoopController();
        rightFeedback = rightMotor.getClosedLoopController();

        SparkMaxConfig config = new SparkMaxConfig();
        config
            .inverted(false)
            .smartCurrentLimit(60)
            .idleMode(IdleMode.kCoast);
        config.encoder
            .positionConversionFactor(positionConversionFactor)
            .velocityConversionFactor(velocityConversionFactor);
        config.closedLoop
            .p(p)
            .i(i)
            .d(d);
        leftMotor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        config
            .follow(leftMotor, true);
        rightMotor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }
}
