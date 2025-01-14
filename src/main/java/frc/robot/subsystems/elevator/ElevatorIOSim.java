package frc.robot.subsystems.elevator;

import static frc.robot.subsystems.elevator.ElevatorConstants.*;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;

public class ElevatorIOSim implements ElevatorIO {
    private ElevatorSim sim = new ElevatorSim(
        LinearSystemId.createElevatorSystem(motor, mass, radius, gearing),
        motor,
        minHeight,
        maxHeight,
        true,
        startingHeight
    );

    private PIDController feedback = new PIDController(p, i, d);
    private double feedforward = 0.0;
    
    @Override
    public void updateInputs(ElevatorIOInputs inputs) {
        double position = sim.getPositionMeters();

        double voltage = feedforward + feedback.calculate(position);
        sim.setInputVoltage(voltage);
        sim.update(0.02);

        double current = sim.getCurrentDrawAmps();

        inputs.position = position;
        inputs.voltages = new double[] {voltage, voltage};
        inputs.currents = new double[] {current, current};
    }

    @Override
    public void setPosition(double position, double ffVoltage) {
        feedforward = ffVoltage;
        feedback.setSetpoint(position);
    }
}
