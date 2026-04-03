package frc.robot.util;

import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.Constants.FieldConstants;

import java.util.function.Supplier;

import static frc.robot.Constants.FieldConstants.BLUE_HUB_CENTER_POSE;

public enum Target {
    IDLE(Translation2d::new),
    HUB(() -> BLUE_HUB_CENTER_POSE.get().getTranslation()),
    DELIVERY(() -> FieldConstants.DELIVERY_LEFT_POSE.get().getTranslation()),
    MANUAL(Translation2d::new);

    private final Supplier<Translation2d> targetSupplier;

    Target(Supplier<Translation2d> targetSupplier) {
        this.targetSupplier = targetSupplier;
    }

    public Translation2d getTargetTranslation() {
        return targetSupplier.get();
    }
}