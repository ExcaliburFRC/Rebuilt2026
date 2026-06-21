package frc.robot.superstructure;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.lib.FieldZones;

import java.util.List;
import java.util.function.Function;

import static frc.robot.superstructure.RobotState.*;

/**
 * Declarative replacement for the dozen hand-wired {@code X_TRIGGER = a.and(b)...onTrue(...)} blocks
 * that used to live in {@code Superstructure}. The full intake x shift x zone x delivery-side decision
 * matrix is expressed as a table of {@link Rule}s; each rule's condition edge-dispatches its target
 * {@link RobotState} through the supplied dispatcher (typically {@code Superstructure::setStateCommand}).
 */
public class RobotStateSelector {

    private record Rule(Trigger when, RobotState then) {}

    public RobotStateSelector(Trigger intakeRequested,
                              Trigger shiftActive,
                              FieldZones zones,
                              Function<RobotState, Command> dispatch) {
        Trigger noIntake = intakeRequested.negate();
        Trigger noShift = shiftActive.negate();
        Trigger inAlliance = zones.inAllianceZone();
        Trigger inNeutral = zones.inNeutralZone();
        Trigger close = zones.closerToCloseDelivery();
        Trigger far = close.negate();

        List<Rule> rules = List.of(
                // Shift active over our hub -> shoot the hub.
                new Rule(noIntake.and(shiftActive).and(inAlliance), NO_INTAKE_SHOOT_HUB),
                new Rule(intakeRequested.and(shiftActive).and(inAlliance), INTAKE_SHOOT_HUB),
                // Shift inactive in the neutral zone -> deliver to the nearer corner.
                new Rule(noIntake.and(noShift).and(inNeutral).and(close), NO_INTAKE_SHOOT_CLOSE_DELIVERY),
                new Rule(noIntake.and(noShift).and(inNeutral).and(far), NO_INTAKE_SHOOT_FAR_DELIVERY),
                new Rule(intakeRequested.and(noShift).and(inNeutral).and(close), INTAKE_SHOOT_CLOSE_DELIVERY),
                new Rule(intakeRequested.and(noShift).and(inNeutral).and(far), INTAKE_SHOOT_FAR_DELIVERY),
                // Shift inactive over our hub -> aim, don't fire yet.
                new Rule(noIntake.and(noShift).and(inAlliance), NO_INTAKE_AIM_HUB),
                new Rule(intakeRequested.and(noShift).and(inAlliance), INTAKE_AIM_HUB),
                // Shift active in the neutral zone -> aim at the delivery corner.
                new Rule(noIntake.and(shiftActive).and(inNeutral).and(close), NO_INTAKE_AIM_CLOSE_DELIVERY),
                new Rule(noIntake.and(shiftActive).and(inNeutral).and(far), NO_INTAKE_AIM_FAR_DELIVERY),
                new Rule(intakeRequested.and(shiftActive).and(inNeutral).and(close), INTAKE_AIM_CLOSE_DELIVERY),
                new Rule(intakeRequested.and(shiftActive).and(inNeutral).and(far), INTAKE_AIM_FAR_DELIVERY)
        );

        for (Rule rule : rules) {
            rule.when().onTrue(dispatch.apply(rule.then()));
        }
    }
}
