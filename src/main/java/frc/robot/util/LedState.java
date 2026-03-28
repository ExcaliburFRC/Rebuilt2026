package frc.robot.util;

import frc.excalib.additional_utilities.Color.Colors;
import frc.excalib.additional_utilities.LEDs.LEDPattern;

import static frc.excalib.additional_utilities.Color.Colors.*;
import static frc.excalib.additional_utilities.Color.Colors.OFF;
import static frc.excalib.additional_utilities.LEDs.LEDPattern.*;


public enum LedState {
    IDLE(EXPAND, TEAM_BLUE, TEAM_GOLD),
    WAITING(EXPAND, RED, TEAM_GOLD),
    LOCKED_ON_TARGET(SOLID, GREEN),
    LOCKED_ON_TARGET_AND_TAG(EXPAND, GREEN, PURPLE),
    OUT_OF_RANGE(BLINKING, RED, ORANGE),
    ONLY_SEES_TAG(EXPAND, TEAM_BLUE, PURPLE);

    public final LEDPattern pattern;
    public final Colors primary,secondary;

    LedState(LEDPattern pattern, Colors primary, Colors secondary){
        this.pattern = pattern;
        this.primary = primary;
        this.secondary = secondary;
    }

    LedState(LEDPattern pattern, Colors primary){
        this.pattern = pattern;
        this.primary = primary;
        this.secondary = OFF;
    }
}
