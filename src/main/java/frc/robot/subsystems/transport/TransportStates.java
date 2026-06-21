package frc.robot.subsystems.transport;

/**
 * Transport drum/feeder states, expressed as the linear velocity (in the FlyWheel's converted units)
 * the rollers should hold.
 */
public enum TransportStates {
    TRANSPORT(20),
    IDLE(0);

    private final double linearVelocity;

    TransportStates(double linearVelocity) {
        this.linearVelocity = linearVelocity;
    }

    /** @return the commanded linear velocity for this state. */
    public double linearVelocity() {
        return linearVelocity;
    }
}
