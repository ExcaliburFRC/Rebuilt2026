package frc.robot.superstructure;

import frc.robot.subsystems.intake.IntakeStates;
import frc.robot.subsystems.shooter.ShooterStates;
import frc.robot.subsystems.transport.TransportStates;

/**
 * A whole-robot state: the tuple of per-subsystem states that {@link Superstructure} dispatches
 * together. The selection logic that decides which state should be active lives in
 * {@link RobotStateSelector}; this enum is a pure aggregate of subsystem goals.
 */
public enum RobotState {
    NO_INTAKE_SHOOT_HUB(IntakeStates.CLOSE, ShooterStates.SHOOT_HUB, TransportStates.TRANSPORT),
    INTAKE_SHOOT_HUB(IntakeStates.OPEN, ShooterStates.SHOOT_HUB, TransportStates.TRANSPORT),
    NO_INTAKE_SHOOT_CLOSE_DELIVERY(IntakeStates.PUMP, ShooterStates.SHOOT_CLOSE_DELIVERY, TransportStates.TRANSPORT),
    NO_INTAKE_SHOOT_FAR_DELIVERY(IntakeStates.PUMP, ShooterStates.SHOOT_FAR_DELIVERY, TransportStates.TRANSPORT),
    INTAKE_SHOOT_CLOSE_DELIVERY(IntakeStates.OPEN, ShooterStates.SHOOT_CLOSE_DELIVERY, TransportStates.TRANSPORT),
    INTAKE_SHOOT_FAR_DELIVERY(IntakeStates.OPEN, ShooterStates.SHOOT_FAR_DELIVERY, TransportStates.TRANSPORT),
    NO_INTAKE_AIM_HUB(IntakeStates.CLOSE, ShooterStates.LOOK_HUB, TransportStates.IDLE),
    INTAKE_AIM_HUB(IntakeStates.OPEN, ShooterStates.LOOK_HUB, TransportStates.IDLE),
    NO_INTAKE_AIM_CLOSE_DELIVERY(IntakeStates.CLOSE, ShooterStates.LOOK_CLOSE_DELIVERY, TransportStates.IDLE),
    NO_INTAKE_AIM_FAR_DELIVERY(IntakeStates.CLOSE, ShooterStates.LOOK_FAR_DELIVERY, TransportStates.IDLE),
    INTAKE_AIM_CLOSE_DELIVERY(IntakeStates.OPEN, ShooterStates.LOOK_CLOSE_DELIVERY, TransportStates.IDLE),
    INTAKE_AIM_FAR_DELIVERY(IntakeStates.OPEN, ShooterStates.LOOK_FAR_DELIVERY, TransportStates.IDLE),
    IDLE(IntakeStates.CLOSE, ShooterStates.IDLE, TransportStates.IDLE);

    private final IntakeStates intakeState;
    private final ShooterStates shooterState;
    private final TransportStates transportState;

    RobotState(IntakeStates intakeState, ShooterStates shooterState, TransportStates transportState) {
        this.intakeState = intakeState;
        this.shooterState = shooterState;
        this.transportState = transportState;
    }

    public IntakeStates intake() {
        return intakeState;
    }

    public ShooterStates shooter() {
        return shooterState;
    }

    public TransportStates transport() {
        return transportState;
    }
}
