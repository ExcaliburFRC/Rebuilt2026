package frc.robot.util;

import edu.wpi.first.math.geometry.Translation3d;
import frc.excalib.additional_utilities.AllianceUtils;
import frc.robot.Constants;

public enum ShootingTarget {
    HUB(Constants.FieldConstants.BLUE_HUB_CENTER_POSE.getAsCurrentAlliance().getTranslation()),
    LEFT_DELIVERY(Constants.FieldConstants.DELIVERY_LEFT_POSE_DISTANCE.getAsCurrentAlliance().getTranslation()),
    RIGHT_DELIVERY(Constants.FieldConstants.DELIVERY_RIGHT_POSE_DIATANCE.getAsCurrentAlliance().getTranslation());

    Translation3d bluePose;

    ShootingTarget(Translation3d blue){
        this.bluePose = blue;
    }

    public Translation3d getTranslation() {
        return AllianceUtils.isBlueAlliance()? bluePose : AllianceUtils.switchAlliance(bluePose);
    }
}
