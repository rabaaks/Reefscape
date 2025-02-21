package frc.robot.subsystems.shooter;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.SparkLowLevel.MotorType;

public class ShooterIOSparkMax implements ShooterIO {
    private final SparkMax motor;

    public ShooterIOSparkMax(int id) {
        motor = new SparkMax(id, MotorType.kBrushless);

        SparkMaxConfig config = new SparkMaxConfig();
        config.smartCurrentLimit(60);
        config.idleMode(IdleMode.kCoast);
        config.inverted(false);
    }

    @Override
    public void updateInputs(ShooterIOInputs inputs) {
        inputs.voltages = new double[] {motor.getAppliedOutput() * motor.getBusVoltage() };
        inputs.currents = new double[] {motor.getOutputCurrent()};
    }

    @Override
    public void setVoltage(double voltage) {
        motor.setVoltage(voltage);
    }
}
