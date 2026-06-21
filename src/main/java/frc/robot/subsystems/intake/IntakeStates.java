package frc.robot.subsystems.intake;

import static frc.robot.subsystems.intake.IntakeConstants.*;

/**
 * Intake states, each carrying its arm goal angle and roller voltage.
 */
public enum IntakeStates {
    OPEN(OPEN_ANGLE, OPEN_ROLLER_VOLTAGE),
    CLOSE(CLOSE_ANGLE, CLOSE_ROLLER_VOLTAGE),
    PUMP(PUMP_ANGLE, PUMP_ROLLER_VOLTAGE);

    private final double angle, voltage;

    IntakeStates(double angle, double voltage) {
        this.angle = angle;
        this.voltage = voltage;
    }

    /** @return the arm goal angle (radians) for this state. */
    public double goalAngle() {
        return angle;
    }

    /** @return the roller voltage for this state. */
    public double rollerVoltage() {
        return voltage;
    }

    /** @return whether this state runs the periodic pump motion rather than a fixed arm goal. */
    public boolean isPumping() {
        return this == PUMP;
    }
}
