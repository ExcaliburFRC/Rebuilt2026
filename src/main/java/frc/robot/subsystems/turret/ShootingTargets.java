package frc.robot.subsystems.turret;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import frc.excalib.additional_utilities.AllianceUtils;
import frc.robot.Constants;

public enum ShootingTargets {
    HUB(Constants.FieldConstants.BLUE_HUB_CENTER_POSE.getAsCurrentAlliance().getTranslation()),
    LEFT_DELIVERY(Constants.FieldConstants.DELIVERY_LEFT_POSE_DISTANCE.getAsCurrentAlliance().getTranslation()),
    RIGHT_DELIVERY(Constants.FieldConstants.DELIVERY_RIGHT_POSE_DIATANCE.getAsCurrentAlliance().getTranslation());

    Translation2d bluePose;

    ShootingTargets(Translation2d blue){
        this.bluePose = blue;
    }

    public Translation2d getTranslation() {
        return AllianceUtils.isBlueAlliance()?
                bluePose : AllianceUtils.switchAlliance(
                        new Translation3d(bluePose)).toTranslation2d();
    }
}
