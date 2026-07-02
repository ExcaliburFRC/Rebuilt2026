package frc.robot.subsystems.shooter;

import edu.wpi.first.math.geometry.Translation2d;
import frc.excalib2.util.AllianceFlip;

import static frc.robot.Constants.FieldConstants.*;
import static frc.robot.subsystems.shooter.TargetHeight.HIGH;
import static frc.robot.subsystems.shooter.TargetHeight.LOW;

public enum ShooterStates {
    SHOOT_HUB(BLUE_HUB_CENTER_POSE, HIGH, true ),
    LOOK_HUB(BLUE_HUB_CENTER_POSE, HIGH, false),
    SHOOT_FAR_DELIVERY(DELIVERY_FAR_POSE, LOW, true),
    LOOK_FAR_DELIVERY(DELIVERY_FAR_POSE, LOW, false),
    SHOOT_CLOSE_DELIVERY(DELIVERY_CLOSE_POSE, LOW, true),
    LOOK_CLOSE_DELIVERY(DELIVERY_CLOSE_POSE, LOW, false),
    IDLE(new AllianceFlip.AllianceTranslation(), LOW, false);

    final AllianceFlip.AllianceTranslation targetTranslation;
    final TargetHeight targetHeight;
    final boolean isShooting;

    ShooterStates(AllianceFlip.AllianceTranslation targetTranslation, TargetHeight targetHeight, boolean isShooting) {
        this.targetTranslation = targetTranslation;
        this.targetHeight = targetHeight;
        this.isShooting = isShooting;
    }

}
