package frc.robot.util;

import edu.wpi.first.wpilibj.DriverStation;
import frc.excalib.additional_utilities.Alliance;


public class GameDataClient {
    public static Alliance OPEN_HUB = Alliance.BOTH;


    public static void updateGameData () {
        String gameData = DriverStation.getGameSpecificMessage();
        if (!gameData.isEmpty()) {
            switch (gameData.charAt(0)) {
                case 'B':
                    OPEN_HUB =  Alliance.BLUE;
                case 'R':
                    OPEN_HUB =  Alliance.RED;
                default:
                    OPEN_HUB = Alliance.BOTH;
            }
        }
    }
}


