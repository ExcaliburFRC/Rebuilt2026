package frc.excalib2.mechanisms;

import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import dev.doglog.DogLog;
import frc.excalib2.device.DeviceConfigs;
import frc.excalib2.device.ExcaTalonFX;
import frc.excalib2.sim.MechanismSim;

import java.util.concurrent.ExecutorService;
import java.util.concurrent.Executors;

import static edu.wpi.first.units.Units.Volts;

/**
 * Base for all mechanism archetypes. Owns everything that is not subsystem-specific:
 *
 * <ul>
 *   <li>device construction + verified configuration (leader, optional follower, optional
 *       fused CANcoder);</li>
 *   <li>disconnect detection (falling-debounced) surfacing as a dashboard {@link Alert}
 *       (pattern credit: 6328 RollerSystem);</li>
 *   <li>DogLog telemetry under the mechanism's name;</li>
 *   <li>physics simulation stepping ({@link MechanismSim});</li>
 *   <li>async brake/coast switching (the CTRE call blocks — never run it on the main loop;
 *       pattern credit: TRIGON, concept only);</li>
 *   <li>SysId routines recording via Phoenix {@code SignalLogger} (CTRE idiom).</li>
 * </ul>
 *
 * <p>The owning subsystem must call {@link #periodic()} from its own {@code periodic()}.
 */
public abstract class Mechanism {
    private static final ExecutorService NEUTRAL_MODE_EXECUTOR = Executors.newSingleThreadExecutor(r -> {
        Thread t = new Thread(r, "excalib2-neutral-mode");
        t.setDaemon(true);
        return t;
    });

    protected final MechanismConfig config;
    protected final ExcaTalonFX motor;
    protected final ExcaTalonFX followerMotor;

    private final Debouncer connectedDebouncer = new Debouncer(0.5, Debouncer.DebounceType.kFalling);
    private final Alert disconnectedAlert;
    private final MechanismSim sim;
    private SysIdRoutine sysIdRoutine;

    protected Mechanism(MechanismConfig config) {
        this.config = config;

        // Fused CANcoder must be configured before the motor references it.
        if (config.cancoderId != null && RobotBase.isReal()) {
            CANcoder cancoder = new CANcoder(config.cancoderId.id(), config.cancoderId.toCANBus());
            DeviceConfigs.applyVerified(config.name + "/CANcoder", cancoder, config.toCANcoderConfiguration());
        }

        motor = new ExcaTalonFX(config.name, config.motorId, config.toTalonFXConfiguration(),
                config.effectiveControlMode());

        if (config.followerId != null) {
            followerMotor = new ExcaTalonFX(config.name + "/Follower", config.followerId,
                    config.toTalonFXConfiguration(), config.effectiveControlMode());
            followerMotor.follow(motor, config.followerOpposed);
        } else {
            followerMotor = null;
        }

        disconnectedAlert = new Alert(config.name + " motor disconnected!", Alert.AlertType.kError);

        if (RobotBase.isSimulation() && config.simModel != null) {
            sim = new MechanismSim(motor.unwrap(), config.simModel.create(),
                    config.rotorPerMechanismRatio(),
                    config.inverted == com.ctre.phoenix6.signals.InvertedValue.Clockwise_Positive);
        } else {
            sim = null;
        }
    }

    /**
     * Housekeeping + telemetry. The owning subsystem calls this once per loop from its
     * {@code periodic()}; signal freshness comes from {@code SignalHub.refreshAll()}.
     */
    public void periodic() {
        if (sim != null) {
            sim.update(TimedRobot.kDefaultPeriod);
        }
        disconnectedAlert.set(!connectedDebouncer.calculate(motor.connected()));

        DogLog.log(config.name + "/PositionRotations", motor.positionRotations());
        DogLog.log(config.name + "/VelocityRPS", motor.velocityRotationsPerSecond());
        DogLog.log(config.name + "/AppliedVolts", motor.volts());
        DogLog.log(config.name + "/SupplyAmps", motor.supplyAmps());
        DogLog.log(config.name + "/StatorAmps", motor.statorAmps());
        DogLog.log(config.name + "/TempCelsius", motor.temperatureCelsius());
        DogLog.log(config.name + "/Connected", motor.connected());
        logExtras();
    }

    /** Archetype-specific telemetry (goals, at-goal flags). */
    protected void logExtras() {
    }

    /** Neutral output (brake/coast per configuration). */
    public void stop() {
        motor.stop();
    }

    /** Switches brake/coast off the main loop (the underlying CTRE call blocks). */
    public void setBrakeAsync(boolean brake) {
        NEUTRAL_MODE_EXECUTOR.execute(() -> {
            NeutralModeValue mode = brake ? NeutralModeValue.Brake : NeutralModeValue.Coast;
            motor.setNeutralMode(mode);
            if (followerMotor != null) {
                followerMotor.setNeutralMode(mode);
            }
        });
    }

    /** Coast while held, restore configured neutral mode on release; runs while disabled. */
    public Command coastCommand(SubsystemBase requirements) {
        return requirements.startEnd(
                        () -> setBrakeAsync(false),
                        () -> setBrakeAsync(config.neutralMode == NeutralModeValue.Brake))
                .ignoringDisable(true)
                .withName(config.name + " Coast");
    }

    /** Direct voltage output — bring-up, SysId, and debugging. */
    public void setVolts(double volts) {
        motor.setVolts(volts);
    }

    public String name() {
        return config.name;
    }

    // ── SysId (records through Phoenix SignalLogger; view hoot in AdvantageScope) ──

    private SysIdRoutine getSysIdRoutine(SubsystemBase subsystem) {
        if (sysIdRoutine == null) {
            sysIdRoutine = new SysIdRoutine(
                    new SysIdRoutine.Config(
                            null, Volts.of(4), null,
                            state -> SignalLogger.writeString(config.name + "/SysIdState", state.toString())),
                    new SysIdRoutine.Mechanism(
                            (Voltage volts) -> motor.setVolts(volts.in(Volts)), null, subsystem));
        }
        return sysIdRoutine;
    }

    public Command sysIdQuasistatic(SubsystemBase subsystem, SysIdRoutine.Direction direction) {
        return getSysIdRoutine(subsystem).quasistatic(direction);
    }

    public Command sysIdDynamic(SubsystemBase subsystem, SysIdRoutine.Direction direction) {
        return getSysIdRoutine(subsystem).dynamic(direction);
    }
}
