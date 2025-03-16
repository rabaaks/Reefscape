package frc.robot.subsystems.roller;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

public class RollerIOTalonSRX implements RollerIO {
    private final TalonSRX topMotor;
    private final TalonSRX bottomMotor;

    public RollerIOTalonSRX(int topId, int bottomId) {
        topMotor = new TalonSRX(topId);
        bottomMotor = new TalonSRX(bottomId);

        topMotor.configPeakCurrentLimit(60);
        bottomMotor.configPeakCurrentLimit(60);

        topMotor.setNeutralMode(NeutralMode.Coast);
        topMotor.setNeutralMode(NeutralMode.Coast);

        topMotor.setInverted(InvertType.InvertMotorOutput);
        bottomMotor.setInverted(InvertType.InvertMotorOutput);

        bottomMotor.follow(topMotor);
    }

    @Override
    public void updateInputs(ShooterIOInputs inputs) {
        inputs.voltages = new double[] {topMotor.getMotorOutputVoltage(), bottomMotor.getMotorOutputVoltage()};
        inputs.currents = new double[] {topMotor.getSupplyCurrent(), bottomMotor.getSupplyCurrent()};
    }

    @Override
    public void setVoltage(double voltage) {
        topMotor.set(ControlMode.PercentOutput, voltage / 12.0);
    }
}
