package frc.robot.subsystems.transport;

public enum TransportStates {
    TRANSPORT(10),
    IDLE(0);

    final double linearVelocity;

    TransportStates(double linearVelocity){
        this.linearVelocity = linearVelocity;
    }
}
