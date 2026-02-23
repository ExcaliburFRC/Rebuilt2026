package frc.robot.util;

import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.Constants;

public enum Target {
    IDLE(new Translation2d()),
    HUB(Constants.FieldConstants.BLUE_HUB_CENTER_POSE.get().getTranslation()),
    DELIVERY(Constants.FieldConstants.DELIVERY_LEFT_POSE.get().getTranslation());

    public Translation2d targetTranslation;

    Target(Translation2d targetTranslation) {
        this.targetTranslation = targetTranslation;
    }
}
