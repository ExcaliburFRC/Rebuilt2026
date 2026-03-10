package frc.robot.util;

import edu.wpi.first.math.geometry.Translation2d;
import frc.excalib.additional_utilities.AllianceUtils;
import frc.robot.Constants.FieldConstants;

import java.util.function.Supplier;

import static frc.robot.Constants.FieldConstants.BLUE_HUB_CENTER_POSE;

public enum Target {
    IDLE(Translation2d::new),
    HUB(() -> {
        if (AllianceUtils.isBlueAlliance()) {
            return BLUE_HUB_CENTER_POSE.get().getTranslation();
        }
        return AllianceUtils.toAlliancePose(BLUE_HUB_CENTER_POSE.get()).getTranslation();
    }),
    DELIVERY(() -> FieldConstants.DELIVERY_LEFT_POSE.get().getTranslation());

    private final Supplier<Translation2d> targetSupplier;

    Target(Supplier<Translation2d> targetSupplier) {
        this.targetSupplier = targetSupplier;
    }

    public Translation2d getTargetTranslation() {
        return targetSupplier.get();
    }
}