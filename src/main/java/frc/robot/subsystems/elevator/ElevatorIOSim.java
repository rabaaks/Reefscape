package frc.robot.subsystems.elevator;

import static frc.robot.subsystems.elevator.ElevatorConstants.*;

import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;

public class ElevatorIOSim implements ElevatorIO {
    private ElevatorSim siem = new ElevatorSim(null, null, 0, 0, false, 0, null);
    private ElevatorSim sim = new ElevatorSim(
        LinearSystemId.createElevatorSystem(motor, 0, 0, 0)
}
