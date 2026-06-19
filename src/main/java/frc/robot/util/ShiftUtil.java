package frc.robot.util;

import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import java.util.ArrayList;
import java.util.List;
import java.util.Optional;
import java.util.function.Consumer;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

/**
 * ShiftUtil — game-data / HUB-shift helper for the 2026 REBUILT game.
 *
 * <p>REBUILT's defining mechanic: during TELEOP, the two ALLIANCE HUBs alternate between
 * active/inactive across four 25-second ALLIANCE SHIFTS. Which alliance starts "inactive" is
 * decided by who scored more Fuel in Auto, and that result is broadcast to both alliances as a
 * single-character FMS Game Specific Message ('R' or 'B') a few seconds into TELEOP. (Game
 * Manual §6.4.1, Table 6-2, Table 6-3.)
 *
 * <p>This class wraps that raw data with: period/shift tracking, a derived "is my hub active"
 * answer usable directly in command bindings, prediction of when the next flip happens (useful
 * for "hold this Fuel a beat longer" logic), event listeners for shift/HUB-state changes,
 * SmartDashboard telemetry, a reproduction of the official driver-station sign code (A/T/R/B/E),
 * and test-injection hooks so the logic can be unit tested without a live DriverStation.
 *
 * <h2>Usage</h2>
 * <pre>{@code
 * // Robot.java
 * @Override public void robotPeriodic() { ShiftUtil.update(); }
 * @Override public void disabledInit() { ShiftUtil.reset(); } // clear cache between matches
 *
 * // Somewhere in a Command or Subsystem
 * if (ShiftUtil.isOwnHubActive()) {
 *     shooter.fire();
 * } else {
 *     intake.holdFuel();
 * }
 *
 * // React to a flip the instant it happens (e.g. drive an LED strip)
 * ShiftUtil.addOwnHubStateListener(active -> leds.setColor(active ? Color.GREEN : Color.RED));
 * }</pre>
 *
 * <p>This class holds static mutable state (cached parsed game data, last-known period, listener
 * lists) by design — there is exactly one match happening at a time on a single robot. It is not
 * thread-safe; call it from the main robot loop, as is standard for WPILib periodic code.
 */
public final class ShiftUtil {

    private ShiftUtil() {
        // static utility class — no instances
    }

    // =========================================================================================
    // Match timing constants (Game Manual Table 6-2)
    // =========================================================================================

    public static final double AUTO_SECONDS = 20.0;
    public static final double TRANSITION_SHIFT_SECONDS = 10.0;
    public static final double SHIFT_SECONDS = 25.0; // each of Shift 1-4
    public static final double END_GAME_SECONDS = 30.0;
    public static final double TELEOP_SECONDS =
            TRANSITION_SHIFT_SECONDS + (4 * SHIFT_SECONDS) + END_GAME_SECONDS; // 140s (2:20)

    /**
     * Boundaries below, measured in "seconds remaining in TELEOP" — i.e. exactly what
     * {@code DriverStation.getMatchTime()} returns once TELEOP has started. A value above
     * {@link #TRANSITION_END} is still the transition shift, above {@link #SHIFT1_END} is Shift
     * 1, and so on down to End Game, which runs to 0.
     */
    private static final double TRANSITION_END = TELEOP_SECONDS - TRANSITION_SHIFT_SECONDS; // 130
    private static final double SHIFT1_END = TRANSITION_END - SHIFT_SECONDS; // 105
    private static final double SHIFT2_END = SHIFT1_END - SHIFT_SECONDS; // 80
    private static final double SHIFT3_END = SHIFT2_END - SHIFT_SECONDS; // 55
    private static final double SHIFT4_END = SHIFT3_END - SHIFT_SECONDS; // 30 (== END_GAME_SECONDS)

    /** Sorted (descending) list of every shift boundary, used when predicting the next flip. */
    private static final double[] ALL_BOUNDARIES =
            {TRANSITION_END, SHIFT1_END, SHIFT2_END, SHIFT3_END, SHIFT4_END, 0.0};

    /** Bonus: point values from Table 6-4, handy for an in-match score estimator. */
    public static final class Scoring {
        private Scoring() {}

        public static final int FUEL_POINTS = 1; // only while the HUB it's scored in is active
        public static final int TOWER_LEVEL1_AUTO = 15;
        public static final int TOWER_LEVEL1_TELEOP = 10;
        public static final int TOWER_LEVEL2_TELEOP = 20;
        public static final int TOWER_LEVEL3_TELEOP = 30;
    }

    // =========================================================================================
    // Period model
    // =========================================================================================

    public enum MatchPeriod {
        PRE_MATCH,
        AUTO,
        TRANSITION_SHIFT,
        SHIFT_1,
        SHIFT_2,
        SHIFT_3,
        SHIFT_4,
        END_GAME,
        /** Match clock data was nonsensical (negative, NaN, etc). Treat conservatively. */
        UNKNOWN
    }

    // =========================================================================================
    // Test-injection hooks (default to the real DriverStation)
    // =========================================================================================

    private static Supplier<String> gameDataSupplier = DriverStation::getGameSpecificMessage;
    private static DoubleSupplier matchTimeSupplier = DriverStation::getMatchTime;
    private static Supplier<Boolean> autoActiveSupplier = DriverStation::isAutonomousEnabled;
    private static Supplier<Boolean> teleopActiveSupplier = DriverStation::isTeleopEnabled;
    private static Supplier<Optional<Alliance>> allianceSupplier = DriverStation::getAlliance;

    /**
     * Redirects every data source this class uses to fakes, so the shift logic can be unit
     * tested off-robot without a live FMS/DriverStation connection. Also clears cached state.
     */
    public static void setTestOverrides(
            Supplier<String> gameData,
            DoubleSupplier matchTime,
            Supplier<Boolean> autoActive,
            Supplier<Boolean> teleopActive,
            Supplier<Optional<Alliance>> alliance) {
        gameDataSupplier = gameData;
        matchTimeSupplier = matchTime;
        autoActiveSupplier = autoActive;
        teleopActiveSupplier = teleopActive;
        allianceSupplier = alliance;
        reset();
    }

    /** Restores the real DriverStation-backed suppliers after a test run. */
    public static void clearTestOverrides() {
        gameDataSupplier = DriverStation::getGameSpecificMessage;
        matchTimeSupplier = DriverStation::getMatchTime;
        autoActiveSupplier = DriverStation::isAutonomousEnabled;
        teleopActiveSupplier = DriverStation::isTeleopEnabled;
        allianceSupplier = DriverStation::getAlliance;
        reset();
    }

    // =========================================================================================
    // Cached state — MUST be cleared between matches, see reset()
    // =========================================================================================

    private static String lastRawGameData = "";
    private static Boolean redInactiveFirstCache = null; // null = not received yet
    private static MatchPeriod lastPeriod = MatchPeriod.PRE_MATCH;
    private static boolean lastOwnHubActive = true;

    private static final List<Consumer<MatchPeriod>> periodChangeListeners = new ArrayList<>();
    private static final List<Consumer<Boolean>> ownHubStateListeners = new ArrayList<>();

    /**
     * Clears every cached value (parsed game data, last period, listener-diff state) without
     * removing registered listeners. <b>Call this from {@code disabledInit()} or at the top of
     * {@code autonomousInit()}</b> — the cached "who's inactive first" result is only valid for
     * the match it was received in, and static fields otherwise leak it into the next match.
     */
    public static void reset() {
        lastRawGameData = "";
        redInactiveFirstCache = null;
        lastPeriod = MatchPeriod.PRE_MATCH;
        lastOwnHubActive = true;
    }

    // =========================================================================================
    // Core period / time queries
    // =========================================================================================

    public static MatchPeriod getCurrentPeriod() {
        if (Boolean.TRUE.equals(autoActiveSupplier.get())) {
            return MatchPeriod.AUTO;
        }
        if (!Boolean.TRUE.equals(teleopActiveSupplier.get())) {
            return MatchPeriod.PRE_MATCH;
        }
        return teleopPeriodFromTime(matchTimeSupplier.getAsDouble());
    }

    /** Maps a raw "seconds remaining in TELEOP" value onto the matching period. */
    private static MatchPeriod teleopPeriodFromTime(double t) {
        if (t > TRANSITION_END) return MatchPeriod.TRANSITION_SHIFT;
        if (t > SHIFT1_END) return MatchPeriod.SHIFT_1;
        if (t > SHIFT2_END) return MatchPeriod.SHIFT_2;
        if (t > SHIFT3_END) return MatchPeriod.SHIFT_3;
        if (t > SHIFT4_END) return MatchPeriod.SHIFT_4;
        if (t >= 0) return MatchPeriod.END_GAME;
        return MatchPeriod.UNKNOWN;
    }

    /** Returns 1-4 during an ALLIANCE SHIFT, otherwise 0. */
    public static int getShiftNumber() {
        switch (getCurrentPeriod()) {
            case SHIFT_1:
                return 1;
            case SHIFT_2:
                return 2;
            case SHIFT_3:
                return 3;
            case SHIFT_4:
                return 4;
            default:
                return 0;
        }
    }

    /** Seconds left in whatever period {@link #getCurrentPeriod()} currently reports. */
    public static double getSecondsRemainingInCurrentPeriod() {
        MatchPeriod p = getCurrentPeriod();
        double t = matchTimeSupplier.getAsDouble();
        switch (p) {
            case AUTO:
            case END_GAME:
                return t;
            case TRANSITION_SHIFT:
                return t - TRANSITION_END;
            case SHIFT_1:
                return t - SHIFT1_END;
            case SHIFT_2:
                return t - SHIFT2_END;
            case SHIFT_3:
                return t - SHIFT3_END;
            case SHIFT_4:
                return t - SHIFT4_END;
            default:
                return Double.NaN;
        }
    }

    /** Total seconds left in the whole MATCH (auto + teleop combined), or NaN before the match starts. */
    public static double getTotalSecondsRemainingInMatch() {
        if (Boolean.TRUE.equals(autoActiveSupplier.get())) {
            return matchTimeSupplier.getAsDouble() + TELEOP_SECONDS;
        }
        if (Boolean.TRUE.equals(teleopActiveSupplier.get())) {
            return matchTimeSupplier.getAsDouble();
        }
        return Double.NaN;
    }

    // =========================================================================================
    // Game data parsing
    // =========================================================================================

    /**
     * Parses (and caches) the FMS Game Specific Message. Returns {@code TRUE} if RED's hub goes
     * inactive first (i.e. RED is inactive in Shift 1 &amp; 3, active in Shift 2 &amp; 4),
     * {@code FALSE} if BLUE is, or {@code null} if the data hasn't arrived yet. Garbage/corrupt
     * single-character payloads are ignored and the last good value (if any) is kept rather than
     * being overwritten — a momentary radio glitch shouldn't flip your strategy mid-shift.
     */
    private static Boolean parseRedInactiveFirst() {
        String raw = gameDataSupplier.get();
        if (raw == null) raw = "";
        if (!raw.equals(lastRawGameData)) {
            lastRawGameData = raw;
            if (!raw.isEmpty()) {
                char c = raw.charAt(0);
                if (c == 'R') {
                    redInactiveFirstCache = Boolean.TRUE;
                } else if (c == 'B') {
                    redInactiveFirstCache = Boolean.FALSE;
                }
                // any other character: corrupt data, keep previous cached value
            }
        }
        return redInactiveFirstCache;
    }

    // =========================================================================================
    // HUB active/inactive queries
    // =========================================================================================

    /** True if {@code alliance}'s HUB is currently scoring points if Fuel goes in. */
    public static boolean isHubActive(Alliance alliance) {
        if (alliance == null) return false;
        return computeHubActiveForAlliance(getCurrentPeriod(), alliance, parseRedInactiveFirst());
    }

    /** Shorthand for {@code isHubActive(DriverStation.getAlliance().orElse(null))}. */
    public static boolean isOwnHubActive() {
        Optional<Alliance> alliance = allianceSupplier.get();
        return alliance.isPresent() && isHubActive(alliance.get());
    }

    /** The mirror image of {@link #isOwnHubActive()} — useful for defense-timing decisions. */
    public static boolean isOpponentHubActive() {
        Optional<Alliance> alliance = allianceSupplier.get();
        if (alliance.isEmpty()) return false;
        Alliance opponent = (alliance.get() == Alliance.Red) ? Alliance.Blue : Alliance.Red;
        return isHubActive(opponent);
    }

    /** True during Auto, the Transition Shift, and End Game — every other period this is false. */
    public static boolean areBothHubsActive() {
        MatchPeriod p = getCurrentPeriod();
        return p == MatchPeriod.AUTO || p == MatchPeriod.TRANSITION_SHIFT || p == MatchPeriod.END_GAME;
    }

    /**
     * Pure function version of the active-HUB logic, parameterized on period/alliance/data so it
     * can be reused for both "right now" queries and "what about at time T" predictions below.
     */
    private static boolean computeHubActiveForAlliance(
            MatchPeriod period, Alliance alliance, Boolean redInactiveFirst) {
        if (period == MatchPeriod.AUTO
                || period == MatchPeriod.TRANSITION_SHIFT
                || period == MatchPeriod.END_GAME) {
            return true;
        }
        if (period == MatchPeriod.PRE_MATCH || period == MatchPeriod.UNKNOWN) {
            return false;
        }
        if (redInactiveFirst == null) {
            // Game data hasn't arrived yet. This window is only ~3 seconds (while Auto Fuel is
            // assessed) and both HUBs are still active during it per the manual, so default true
            // rather than guessing wrong and bricking a scoring opportunity.
            return true;
        }
        boolean shift1ActiveForRed = !redInactiveFirst;
        boolean activeForRed;
        switch (period) {
            case SHIFT_1:
                activeForRed = shift1ActiveForRed;
                break;
            case SHIFT_2:
                activeForRed = !shift1ActiveForRed;
                break;
            case SHIFT_3:
                activeForRed = shift1ActiveForRed;
                break;
            case SHIFT_4:
                activeForRed = !shift1ActiveForRed;
                break;
            default:
                return true; // unreachable given the guard above
        }
        return (alliance == Alliance.Red) == activeForRed;
    }

    // =========================================================================================
    // Prediction — "how long until my HUB flips"
    // =========================================================================================

    /**
     * Seconds until your own HUB's active/inactive state next changes, or {@link Double#NaN} if
     * that can't be determined yet (pre-match, Auto, or game data not received) or there are no
     * more flips left this match. Handy for decisions like "I have 1 Fuel and 4 seconds left in
     * this inactive window — worth holding for the next active shift instead of wasting the shot."
     */
    public static double getSecondsUntilOwnHubFlips() {
        MatchPeriod currentPeriod = getCurrentPeriod();
        boolean inAShiftWindow =
                currentPeriod == MatchPeriod.TRANSITION_SHIFT
                        || currentPeriod == MatchPeriod.SHIFT_1
                        || currentPeriod == MatchPeriod.SHIFT_2
                        || currentPeriod == MatchPeriod.SHIFT_3
                        || currentPeriod == MatchPeriod.SHIFT_4;
        if (!inAShiftWindow) {
            return Double.NaN;
        }

        Optional<Alliance> allianceOpt = allianceSupplier.get();
        Boolean redInactiveFirst = parseRedInactiveFirst();
        if (allianceOpt.isEmpty() || redInactiveFirst == null) {
            return Double.NaN;
        }

        Alliance alliance = allianceOpt.get();
        double t = matchTimeSupplier.getAsDouble();
        boolean currentlyActive = computeHubActiveForAlliance(currentPeriod, alliance, redInactiveFirst);

        for (double boundary : ALL_BOUNDARIES) {
            if (boundary >= t) continue; // already in the past relative to "now"
            MatchPeriod periodJustAfter = teleopPeriodFromTime(Math.max(boundary - 0.001, 0));
            boolean activeJustAfter = computeHubActiveForAlliance(periodJustAfter, alliance, redInactiveFirst);
            if (activeJustAfter != currentlyActive) {
                return t - boundary;
            }
        }
        return Double.NaN; // no further flips before the match ends
    }

    // =========================================================================================
    // Driver-station sign code (Game Manual §5.9.1) — reproduce it on a custom dashboard
    // =========================================================================================

    /**
     * Reproduces the single-character code shown on the back of the team sign: 'A' during Auto,
     * 'T' during the Transition Shift, 'E' during End Game, or 'R'/'B' during a numbered shift
     * depending on which alliance's HUB is active. Returns '?' if game data hasn't arrived yet,
     * or '-' pre-match.
     */
    public static char getDriverStationSignCode() {
        MatchPeriod p = getCurrentPeriod();
        switch (p) {
            case AUTO:
                return 'A';
            case TRANSITION_SHIFT:
                return 'T';
            case END_GAME:
                return 'E';
            case SHIFT_1:
            case SHIFT_2:
            case SHIFT_3:
            case SHIFT_4:
                Boolean redInactiveFirst = parseRedInactiveFirst();
                if (redInactiveFirst == null) return '?';
                boolean redActive = computeHubActiveForAlliance(p, Alliance.Red, redInactiveFirst);
                return redActive ? 'R' : 'B';
            default:
                return '-';
        }
    }

    // =========================================================================================
    // Listeners + periodic update/telemetry
    // =========================================================================================

    /** Fired once, exactly when {@link #getCurrentPeriod()} changes (e.g. Shift 2 -> Shift 3). */
    public static void addPeriodChangeListener(Consumer<MatchPeriod> listener) {
        periodChangeListeners.add(listener);
    }

    /** Fired once, exactly when your own HUB flips active <-> inactive. */
    public static void addOwnHubStateListener(Consumer<Boolean> listener) {
        ownHubStateListeners.add(listener);
    }

    public static void clearListeners() {
        periodChangeListeners.clear();
        ownHubStateListeners.clear();
    }

    /**
     * Call once per loop (e.g. from {@code Robot.robotPeriodic()}). Detects period/HUB-state
     * transitions to fire listeners, and pushes a small telemetry block to SmartDashboard.
     */
    public static void update() {
        MatchPeriod current = getCurrentPeriod();
        if (current != lastPeriod) {
            lastPeriod = current;
            for (Consumer<MatchPeriod> listener : periodChangeListeners) {
                listener.accept(current);
            }
        }

        boolean ownActive = isOwnHubActive();
        if (ownActive != lastOwnHubActive) {
            lastOwnHubActive = ownActive;
            for (Consumer<Boolean> listener : ownHubStateListeners) {
                listener.accept(ownActive);
            }
        }

        publishTelemetry(current, ownActive);
    }

    private static void publishTelemetry(MatchPeriod period, boolean ownHubActive) {
        SmartDashboard.putString("Shift/Period", period.toString());
        SmartDashboard.putNumber("Shift/Number", getShiftNumber());
        SmartDashboard.putString("Shift/SignCode", String.valueOf(getDriverStationSignCode()));
        SmartDashboard.putBoolean("Shift/OwnHubActive", ownHubActive);
        SmartDashboard.putBoolean("Shift/OpponentHubActive", isOpponentHubActive());
        SmartDashboard.putNumber("Shift/SecondsLeftInPeriod", getSecondsRemainingInCurrentPeriod());
        SmartDashboard.putNumber("Shift/SecondsUntilOwnHubFlips", getSecondsUntilOwnHubFlips());
    }

    /** One-line human-readable snapshot, handy for console logging / driver feedback. */
    public static String getDebugString() {
        return String.format(
                "Period=%s Shift=%d Sign=%c OwnHub=%s OppHub=%s TimeLeftInPeriod=%.1fs FlipsIn=%.1fs",
                getCurrentPeriod(),
                getShiftNumber(),
                getDriverStationSignCode(),
                isOwnHubActive(),
                isOpponentHubActive(),
                getSecondsRemainingInCurrentPeriod(),
                getSecondsUntilOwnHubFlips());
    }
}