package frc.robot.subsystems.turret;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import frc.excalib.additional_utilities.AllianceUtils;
import frc.robot.Constants;
import frc.robot.Constants.FieldConstants.*;

public enum ShootingTargets {
    HUB(Constants.FieldConstants.BLUE_HUB_CENTER_POSE.get().getTranslation()),
    LEFT_DELIVERY(Constants.FieldConstants.BLUE_LEFT_DELIVERY_SIDE.get().getTranslation()),
    RIGHT_DELIVERY(Constants.FieldConstants.BLUE_RIGHT_DELIVERY_SIDE.get().getTranslation());

    Translation2d blue;

    ShootingTargets(Translation2d blue){
        this.blue = blue;

    }

    public Translation2d getTranslation() {
        return AllianceUtils.isBlueAlliance()? blue : AllianceUtils.switchAlliance(new Translation3d(blue)).toTranslation2d();
    }
}
