package frc.robot.subsystems.elevator;

import static frc.robot.subsystems.elevator.ElevatorConstants.*;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;

public class ElevatorIOSim implements ElevatorIO {

    private ElevatorSim sim;

    private PIDController feedback;
    private double feedforward = 0.0;

    private IOConfig config;

    public ElevatorIOSim(IOConfig ioConfig, SimConfig config) {
        sim = new ElevatorSim(
            LinearSystemId.createElevatorSystem(config.motor(), config.mass(), ioConfig.radius(), ioConfig.gearing()),
            config.motor(),
            ioConfig.minHeight(),
            ioConfig.maxHeight(),
            true,
            ioConfig.minHeight()
        );

        feedback = new PIDController(ioConfig.p(), ioConfig.i(), ioConfig.d());
    }
    
    @Override
    public void updateInputs(ElevatorIOInputs inputs) {
        double position = sim.getPositionMeters();

        double output = MathUtil.clamp(feedforward + feedback.calculate(position), -1.0, 1.0);
        sim.setInputVoltage(output * 12.0);
        sim.update(0.02);

        inputs.position = position;
        inputs.velocity = sim.getVelocityMetersPerSecond();
        inputs.outputs = new double[] {output};
        inputs.currents = new double[] {sim.getCurrentDrawAmps()};
    }

    @Override
    public void setPosition(double position, double ffVoltage) {
        feedforward = ffVoltage;
        feedback.setSetpoint(position);
    }
}
