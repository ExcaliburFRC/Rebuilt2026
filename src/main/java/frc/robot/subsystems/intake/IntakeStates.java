package frc.robot.subsystems.intake;

public enum IntakeStates {
    OPEN(0, 0.75),
    CLOSE(2.2, 0);
    final double angle, voltage;

    IntakeStates(double angle, double voltage) {
        this.angle = angle;
        this.voltage = voltage;
    }
}
