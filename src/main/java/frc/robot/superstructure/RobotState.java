package frc.robot.superstructure;

import frc.robot.subsystems.intake.IntakeStates;
import frc.robot.subsystems.shooter.ShooterStates;
import frc.robot.subsystems.transport.TransportStates;

public enum RobotState {
    NO_INTAKE_SHOOT_HUB(IntakeStates.CLOSE, ShooterStates.SHOOT_HUB, TransportStates.TRANSPORT),
    INTAKE_SHOOT_HUB(IntakeStates.OPEN,ShooterStates.SHOOT_HUB,TransportStates.TRANSPORT),
    NO_INTAKE_SHOOT_CLOSE_DELIVERY(IntakeStates.CLOSE, ShooterStates.SHOOT_CLOSE_DELIVERY,TransportStates.TRANSPORT),
    NO_INTAKE_SHOOT_FAR_DELIVERY(IntakeStates.CLOSE, ShooterStates.SHOOT_FAR_DELIVERY,TransportStates.TRANSPORT),
    INTAKE_SHOOT_CLOSE_DELIVERY(IntakeStates.OPEN, ShooterStates.SHOOT_CLOSE_DELIVERY,TransportStates.TRANSPORT),
    INTAKE_SHOOT_FAR_DELIVERY(IntakeStates.OPEN, ShooterStates.SHOOT_FAR_DELIVERY,TransportStates.TRANSPORT),
    NO_INTAKE_AIM_HUB(IntakeStates.CLOSE, ShooterStates.LOOK_HUB,TransportStates.IDLE),
    INTAKE_AIM_HUB(IntakeStates.OPEN, ShooterStates.LOOK_HUB,TransportStates.IDLE),
    NO_INTAKE_AIM_CLOSE_DELIVERY(IntakeStates.CLOSE, ShooterStates.LOOK_CLOSE_DELIVERY,TransportStates.IDLE),
    NO_INTAKE_AIM_FAR_DELIVERY(IntakeStates.CLOSE, ShooterStates.LOOK_FAR_DELIVERY,TransportStates.IDLE),
    INTAKE_AIM_CLOSE_DELIVERY(IntakeStates.OPEN, ShooterStates.LOOK_CLOSE_DELIVERY,TransportStates.IDLE),
    INTAKE_AIM_FAR_DELIVERY(IntakeStates.OPEN, ShooterStates.LOOK_FAR_DELIVERY,TransportStates.IDLE),
    IDLE(IntakeStates.CLOSE, ShooterStates.IDLE,TransportStates.IDLE);

    IntakeStates intakeState;
    ShooterStates shooterState;
    TransportStates transportState;

    RobotState(IntakeStates intakeState, ShooterStates shooterState, TransportStates transportState) {
        this.intakeState = intakeState;
        this.shooterState = shooterState;
        this.transportState = transportState;
    }
}
