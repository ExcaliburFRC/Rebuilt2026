package frc.excalib2.device;

import com.ctre.phoenix6.CANBus;

/**
 * A CAN device identity: id + bus name, as one value.
 *
 * <p>Prevents the classic "right id, wrong bus" mistake by keeping the two inseparable.
 * (Pattern credit: Team 254 {@code CANDeviceId}.)
 */
public record CANDeviceId(int id, String busName) {

    /** Device on the roboRIO CAN bus. */
    public CANDeviceId(int id) {
        this(id, "");
    }

    public CANBus toCANBus() {
        return new CANBus(busName);
    }

    @Override
    public String toString() {
        return busName.isEmpty() ? ("rio[" + id + "]") : (busName + "[" + id + "]");
    }
}
