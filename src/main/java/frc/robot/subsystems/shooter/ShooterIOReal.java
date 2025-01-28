package frc.robot.subsystems.shooter;

import static frc.robot.subsystems.shooter.ShooterConstants.*;

import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.TalonSRXConfiguration;

public class ShooterIOReal implements ShooterIO {
    private final TalonSRX topMotor;
    private final TalonSRX bottomMotor;

    public ShooterIOReal(int topId, int bottomId) {
        topMotor = new TalonSRX(topId);
        bottomMotor = new TalonSRX(bottomId);

        topMotor.configPeakCurrentLimit(60);
        bottomMotor.configPeakCurrentLimit(60);

        topMotor.setNeutralMode(NeutralMode.Coast);
        topMotor.setNeutralMode(NeutralMode.Coast);

        bottomMotor.follow(topMotor);

        bottomMotor.setInverted(InvertType.OpposeMaster);
    }

    @Override
    public void updateInputs(ShooterIOInputs inputs) {
        inputs.voltages = new double[] {topMotor.getMotorOutputVoltage, bottomMotor.getMotorOutputVoltage()}
        inputs.leftCurrent = leftMotor.getOutputCurrent();
        inputs.rightCurrent = rightMotor.getOutputCurrent();
    }

    @Override
    public void setVoltages(double leftVoltage, double rightVoltage) {
        leftMotor.setVoltage(leftVoltage);
        rightMotor.setVoltage(rightVoltage);
    }
}
