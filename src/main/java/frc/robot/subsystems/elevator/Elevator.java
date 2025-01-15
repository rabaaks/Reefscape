package frc.robot.subsystems.elevator;

import static frc.robot.subsystems.elevator.ElevatorConstants.*;

import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Elevator extends SubsystemBase {
    private final ElevatorIO io;
    private final ElevatorIOInputsAutoLogged inputs = new ElevatorIOInputsAutoLogged();

    private final TrapezoidProfile profile = new TrapezoidProfile(new TrapezoidProfile.Constraints(maxProfileVelocity, maxProfileAcceleration));
    private TrapezoidProfile.State profileState = new TrapezoidProfile.State(0, 0);
    private TrapezoidProfile.State futureProfileState = new TrapezoidProfile.State(0, 0);

    private final ElevatorFeedforward feedforward = new ElevatorFeedforward(s, g, v, a);

    public Elevator(ElevatorIO io) {
        this.io = io;
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("Elevator", inputs);
    }

    public void setPosition(double position) {
        futureProfileState = profile.calculate(0.02, profileState, new TrapezoidProfile.State(position, 0.0));
        Logger.recordOutput("Elevator/PositionSetpoint", profileState.position);
        Logger.recordOutput("Elevator/VelocitySetpoint", profileState.velocity);
        io.setPosition(position, feedforward.calculateWithVelocities(profileState.velocity, futureProfileState.velocity));
        profileState = futureProfileState;
    }

    @AutoLogOutput(key="Odometry/ElevatorPosition")
    public double getPosition() {
        return inputs.position;
    }

    @AutoLogOutput(key="Odometry/ElevatorVelocity")
    public double getVelocity() {
        return inputs.velocity;
    }
}
