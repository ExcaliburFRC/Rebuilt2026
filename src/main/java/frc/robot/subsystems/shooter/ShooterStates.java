package frc.robot.subsystems.shooter;

import frc.excalib.additional_utilities.AllianceUtils;

import static frc.robot.Constants.FieldConstants.*;
import static frc.robot.subsystems.shooter.TargetHeight.HIGH;
import static frc.robot.subsystems.shooter.TargetHeight.LOW;

/**
 * Shooter states: each names a target translation, the goal height (which selects the HIGH/LOW
 * interpolation tables), and whether the flywheel should actually spin up to shoot.
 */
public enum ShooterStates {
    SHOOT_HUB(BLUE_HUB_CENTER_POSE, HIGH, true),
    LOOK_HUB(BLUE_HUB_CENTER_POSE, HIGH, false),
    SHOOT_FAR_DELIVERY(DELIVERY_FAR_POSE, LOW, true),
    LOOK_FAR_DELIVERY(DELIVERY_FAR_POSE, LOW, false),
    SHOOT_CLOSE_DELIVERY(DELIVERY_CLOSE_POSE, LOW, true),
    LOOK_CLOSE_DELIVERY(DELIVERY_CLOSE_POSE, LOW, false),
    IDLE(new AllianceUtils.AllianceTranslation(), LOW, false);

    private final AllianceUtils.AllianceTranslation targetTranslation;
    private final TargetHeight targetHeight;
    private final boolean isShooting;

    ShooterStates(AllianceUtils.AllianceTranslation targetTranslation, TargetHeight targetHeight, boolean isShooting) {
        this.targetTranslation = targetTranslation;
        this.targetHeight = targetHeight;
        this.isShooting = isShooting;
    }

    /** @return the alliance-relative target this state aims at. */
    public AllianceUtils.AllianceTranslation target() {
        return targetTranslation;
    }

    /** @return the goal height for this state. */
    public TargetHeight targetHeight() {
        return targetHeight;
    }

    /** @return whether the flywheel should spin to a shooting velocity in this state. */
    public boolean isShooting() {
        return isShooting;
    }

    /** @return whether this state targets the HIGH goal (selects the HIGH interpolation tables). */
    public boolean isHighGoal() {
        return targetHeight == HIGH;
    }
}
