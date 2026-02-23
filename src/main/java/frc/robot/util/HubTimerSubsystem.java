package frc.robot.util;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class HubTimerSubsystem extends SubsystemBase {
    public static final double HUB_PERIOD = 30.0;
    public static final double HUB_ACTIVE_TIME = 10.0;


    @Override
    public void periodic() {
        double matchTime = DriverStation.getMatchTime();


        SmartDashboard.putString(
                "Alliance",
                DriverStation.getAlliance().map(Enum::name).orElse("Unknown")
        );


        if (matchTime < 0) {
            SmartDashboard.putBoolean("HubActive", false);
            SmartDashboard.putNumber("HubTimeRemaining", 0);
            return;
        }

        double timeIntoCycle = matchTime % HUB_PERIOD;
        boolean hubActive = timeIntoCycle <= HUB_ACTIVE_TIME;

        double timeRemaining = hubActive
                ? HUB_ACTIVE_TIME - timeIntoCycle
                : HUB_PERIOD - timeIntoCycle;

        SmartDashboard.putBoolean("HubActive", hubActive);
        SmartDashboard.putNumber("HubTimeRemaining", timeRemaining);
    }
}
