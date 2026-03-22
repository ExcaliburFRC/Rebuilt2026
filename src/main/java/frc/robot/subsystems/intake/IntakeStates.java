package frc.robot.subsystems.intake;

public enum IntakeStates {
    OPEN(1, -9),
    CLOSE(0, 0);
    final double angle, voltage;

    IntakeStates(double angle, double voltage) {
        this.angle = angle;
        this.voltage = voltage;
    }
}
