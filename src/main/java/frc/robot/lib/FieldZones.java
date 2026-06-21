package frc.robot.lib;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.excalib.additional_utilities.AllianceUtils;
import monologue.Annotations.Log;
import monologue.Logged;

import java.util.function.Supplier;

import static frc.excalib.additional_utilities.AllianceUtils.FIELD_LENGTH_METERS;
import static frc.excalib.additional_utilities.AllianceUtils.FIELD_WIDTH_METERS;
import static frc.robot.Constants.FieldConstants.*;

/**
 * Single owner of all field-geometry zone queries, expressed as named {@link Trigger}s.
 * <p>
 * Centralizes logic that previously lived split between {@code Superstructure} (alliance/neutral
 * zones, delivery side, bump routing) and {@code Shooter} (the trench detection that forces the hood
 * down). Each consumer constructs a {@code FieldZones} over whichever pose reference is meaningful to
 * it (robot pose for the hood-trench check, turret-on-field for delivery routing), so the geometry
 * math is defined once here while each caller keeps its own reference frame.
 */
public class FieldZones implements Logged {
    private final Supplier<Pose2d> reference;

    private final Trigger inAllianceZone;
    private final Trigger inIntermediateZone;
    private final Trigger inNeutralZone;
    private final Trigger closerToCloseDelivery;
    private final Trigger underTrench;
    private final Trigger overBump;
    private final Trigger inTrench;

    public FieldZones(Supplier<Pose2d> reference) {
        this.reference = reference;

        // Alliance / intermediate zone detection is currently forced on the team's robot (the real
        // geometry-based triggers were disabled). Behavior is preserved here; the real implementation
        // can be restored by replacing these two suppliers.
        inAllianceZone = new Trigger(() -> true);
        inIntermediateZone = new Trigger(() -> false);
        inNeutralZone = inAllianceZone.or(inIntermediateZone).negate();

        closerToCloseDelivery = new Trigger(() -> y() < FIELD_WIDTH_METERS / 2);

        underTrench = new Trigger(() -> {
            double y = y();
            return y < CLOSE_TRENCH_TO_BUMP_X || y > FAR_TRENCH_TO_BUMP_X;
        }).and(inIntermediateZone);

        overBump = inIntermediateZone.and(underTrench.negate());

        inTrench = new Trigger(this::computeInTrench);
    }

    private double y() {
        return reference.get().getY();
    }

    /** Whether the reference pose sits physically inside the trench (hood must stay low). */
    private boolean computeInTrench() {
        Pose2d pose = reference.get();
        if (AllianceUtils.isBlueAlliance()) {
            return pose.getX() > FRONT_TRENCH_SIDEX_LINE_DIST_METERS
                    && pose.getY() < TRENCH_SIDEY_LINE_DIST_METERS
                    && pose.getX() < BACK_TRENCH_SIDEX_LINE_DIST_METERS;
        }
        return pose.getX() < FIELD_LENGTH_METERS - FRONT_TRENCH_SIDEX_LINE_DIST_METERS
                && pose.getY() > FIELD_WIDTH_METERS - TRENCH_SIDEY_LINE_DIST_METERS
                && pose.getX() > FIELD_LENGTH_METERS - BACK_TRENCH_SIDEX_LINE_DIST_METERS;
    }

    public Trigger inAllianceZone() {
        return inAllianceZone;
    }

    public Trigger inIntermediateZone() {
        return inIntermediateZone;
    }

    public Trigger inNeutralZone() {
        return inNeutralZone;
    }

    public Trigger closerToCloseDelivery() {
        return closerToCloseDelivery;
    }

    public Trigger underTrench() {
        return underTrench;
    }

    public Trigger overBump() {
        return overBump;
    }

    public Trigger inTrench() {
        return inTrench;
    }

    @Log.NT
    public boolean getInAllianceZone() {
        return inAllianceZone.getAsBoolean();
    }

    @Log.NT
    public boolean getInIntermediateZone() {
        return inIntermediateZone.getAsBoolean();
    }

    @Log.NT
    public boolean getInNeutralZone() {
        return inNeutralZone.getAsBoolean();
    }

    @Log.NT
    public boolean getCloserToCloseDelivery() {
        return closerToCloseDelivery.getAsBoolean();
    }

    @Log.NT
    public boolean getInTrench() {
        return inTrench.getAsBoolean();
    }
}
