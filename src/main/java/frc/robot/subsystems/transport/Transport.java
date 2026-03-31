package frc.robot.subsystems.transport;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.excalib.mechanisms.fly_wheel.FlyWheel;
import monologue.Logged;
import org.littletonrobotics.junction.Logger;

import java.util.function.DoubleSupplier;

import static frc.robot.subsystems.transport.TransportConstants.*;

public class Transport extends SubsystemBase implements Logged {
    private static final double TRANSPORT_FUEL_VOLTAGE = -3.0;

    private final TransportIO io;
    private final TransportIOInputsAutoLogged inputs = new TransportIOInputsAutoLogged();

    public FlyWheel drumMechanism;

    public Transport(TransportIO io) {
        this.io = io;
        drumMechanism = new FlyWheel(io.getDrumMotor(), MAX_ACCELERATION, MAX_JERK, GAINS);

        setDefaultCommand(transportFuelCommand());
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("Transport", inputs);
    }

    public Command manualCommand(DoubleSupplier output) {
        return drumMechanism.manualCommand(output, this);
    }

    public Command transportFuelCommand() {
        return drumMechanism.manualCommand(() -> TRANSPORT_FUEL_VOLTAGE, this);
    }
}
