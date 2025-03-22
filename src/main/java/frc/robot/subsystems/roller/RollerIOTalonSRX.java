package frc.robot.subsystems.roller;

import static frc.robot.subsystems.roller.RollerConstants.*;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

public class RollerIOTalonSRX implements RollerIO {
    private final TalonSRX topMotor;
    private final TalonSRX bottomMotor;

    public RollerIOTalonSRX(RealConfig config) {
        topMotor = new TalonSRX(config.topMotorId());
        bottomMotor = new TalonSRX(config.bottomMotorId());

        topMotor.configPeakCurrentLimit(60);
        bottomMotor.configPeakCurrentLimit(60);

        topMotor.setNeutralMode(NeutralMode.Coast);
        topMotor.setNeutralMode(NeutralMode.Coast);

        topMotor.setInverted(InvertType.InvertMotorOutput);
        bottomMotor.setInverted(InvertType.InvertMotorOutput);

        bottomMotor.follow(topMotor);
    }

    @Override
    public void updateInputs(RollerIOInputs inputs) {
        inputs.outputs = new double[] {topMotor.getMotorOutputPercent(), bottomMotor.getMotorOutputPercent()};
        inputs.currents = new double[] {topMotor.getSupplyCurrent(), bottomMotor.getSupplyCurrent()};
    }

    @Override
    public void set(double output) {
        topMotor.set(ControlMode.PercentOutput, output);
    }
}
