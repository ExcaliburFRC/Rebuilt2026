package frc.robot.subsystems.intake;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.excalib2.control.ControlMode;
import frc.excalib2.device.CANDeviceId;
import frc.excalib2.mechanisms.MechanismConfig;
import frc.excalib2.mechanisms.PositionalMechanism;
import frc.excalib2.mechanisms.RollerMechanism;
import frc.excalib2.statemachine.StateMachine;

import static edu.wpi.first.units.Units.Radians;
import static frc.robot.subsystems.intake.IntakeConstants.*;
import static frc.robot.subsystems.intake.IntakeStates.*;

/**
 * Intake on ExcaLib v2: four-bar as a {@link PositionalMechanism} (MotionMagic +
 * arm-cosine gravity FF, both motors leader/follower) and the roller as a
 * {@link RollerMechanism}, coordinated by a three-state machine (OPEN / CLOSE / PUMP —
 * PUMP oscillates the four-bar while running the roller, as in v1).
 */
public class Intake extends SubsystemBase {

    private final PositionalMechanism fourBar = new PositionalMechanism(
            MechanismConfig.of("Intake/FourBar", new CANDeviceId(LEFT_FOUR_BAR_MOTOR_ID))
                    .follower(new CANDeviceId(RIGHT_FOUR_BAR_MOTOR_ID), true)
                    .controlMode(ControlMode.VOLTAGE) // ported v1 volts gains; FOC after SysId
                    .rotorToMechanismRatio(GEARING)
                    .gains(FOUR_BAR_GAINS, FOUR_BAR_SIM_GAINS)
                    .motion(FOUR_BAR_MOTION)
                    .softLimits(Radians.of(INTAKE_MIN_ANGLE_RAD), Radians.of(INTAKE_MAX_ANGLE_RAD))
                    .peakVoltage(FOUR_BAR_PEAK_VOLTS, -FOUR_BAR_PEAK_VOLTS)
                    .currentBudget(FOUR_BAR_BUDGET)
                    .tolerance(Radians.of(INTAKE_ANGLE_TOLERANCE_RAD))
                    .neutralMode(com.ctre.phoenix6.signals.NeutralModeValue.Coast)
                    .simModel(FOUR_BAR_SIM_MODEL));

    private final RollerMechanism roller = new RollerMechanism(
            MechanismConfig.of("Intake/Roller", new CANDeviceId(ROLLER_MOTOR_ID))
                    .controlMode(ControlMode.VOLTAGE)
                    .currentBudget(ROLLER_BUDGET)
                    .simModel(ROLLER_SIM_MODEL));

    private final StateMachine<IntakeStates> machine;

    /** Four-bar within tolerance of the current state's angle (v1 semantics). */
    public final Trigger atPositionTrigger;

    public Intake() {
        // v1 seeded the arm position from a constant at boot (CANcoder present but unused —
        // fusing it needs an on-robot offset calibration; planned follow-up).
        fourBar.resetPosition(Radians.of(START_ANGLE_RAD));

        machine = StateMachine.builder("Intake", CLOSE)
                .whileIn(OPEN, run(() -> {
                    fourBar.setGoal(Radians.of(OPEN.angle));
                    roller.runDutyCycle(OPEN.voltage);
                }))
                .whileIn(CLOSE, run(() -> {
                    fourBar.setGoal(Radians.of(CLOSE.angle));
                    roller.runDutyCycle(CLOSE.voltage);
                }))
                .whileIn(PUMP, pumpCommand())
                .transitionFromAny(OPEN)
                .transitionFromAny(CLOSE)
                .transitionFromAny(PUMP)
                .build();

        atPositionTrigger = new Trigger(() -> Math.abs(
                machine.getCurrentState().angle - fourBar.getPosition().in(Radians))
                < INTAKE_ANGLE_TOLERANCE_RAD);
    }

    /** PUMP: roller feeds while the four-bar oscillates high/low every half second (v1 behavior). */
    private Command pumpCommand() {
        return Commands.repeatingSequence(
                        run(() -> {
                            fourBar.setGoal(Radians.of(PUMP_HIGH_ANGLE_RAD));
                            roller.runDutyCycle(PUMP.voltage);
                        }).withTimeout(PUMP_PERIOD_SECONDS),
                        run(() -> {
                            fourBar.setGoal(Radians.of(PUMP_LOW_ANGLE_RAD));
                            roller.runDutyCycle(PUMP.voltage);
                        }).withTimeout(PUMP_PERIOD_SECONDS))
                .withName("Intake Pump");
    }

    public Command setStateCommand(IntakeStates state) {
        return machine.requestCommand(state);
    }

    public Command coastCommand() {
        return fourBar.coastCommand(this);
    }

    @Override
    public void periodic() {
        fourBar.periodic();
        roller.periodic();
        machine.periodic();
    }
}
