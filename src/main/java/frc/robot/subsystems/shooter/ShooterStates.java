package frc.robot.subsystems.shooter;

import edu.wpi.first.math.geometry.Translation2d;
import frc.excalib.additional_utilities.AllianceUtils;

import static frc.robot.Constants.FieldConstants.*;
import static frc.robot.subsystems.shooter.TargetHeight.HIGH;
import static frc.robot.subsystems.shooter.TargetHeight.LOW;

public enum ShooterStates {
    SHOOT_HUB(BLUE_HUB_CENTER_POSE.get().getTranslation(), HIGH, true ),
    LOOK_HUB(BLUE_HUB_CENTER_POSE.get().getTranslation(), HIGH, false),
    SHOOT_FAR_DELIVERY(DELIVERY_FAR_POSE.get().getTranslation(), LOW, true),
    LOOK_FAR_DELIVERY(DELIVERY_FAR_POSE.get().getTranslation(), LOW, false),
    SHOOT_CLOSE_DELIVERY(DELIVERY_CLOSE_POSE.get().getTranslation(), LOW, true),
    LOOK_CLOSE_DELIVERY(DELIVERY_CLOSE_POSE.get().getTranslation(), LOW, false),
    IDLE(new Translation2d(), LOW, false);

    final Translation2d targetTranslation;
    final TargetHeight targetHeight;
    final boolean isShooting;

    ShooterStates(Translation2d blueTargetTranslation, TargetHeight targetHeight, boolean isShooting) {
        this.targetTranslation = blueTargetTranslation;
        this.targetHeight = targetHeight;
        this.isShooting = isShooting;
    }

}
