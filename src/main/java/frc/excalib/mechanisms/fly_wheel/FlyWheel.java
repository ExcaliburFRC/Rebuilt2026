package frc.excalib.mechanisms.fly_wheel;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.excalib.control.gains.Gains;
import frc.excalib.control.motor.controllers.Motor;
import frc.excalib.mechanisms.Mechanism;

import java.util.function.DoubleSupplier;

public class FlyWheel extends Mechanism {

    private final PIDController              m_pid;
    private final SimpleMotorFeedforward     m_ff;
    private final Gains                      m_gains;
    private final TrapezoidProfile           m_profile;

    // ── Acceleration estimation ───────────────────────────────────────────────
    //
    // Two modes:
    //
    //   CURRENT-BASED (preferred):
    //     Physics: τ = kt × I_stator,  α = τ / J  →  α = (kt/J) × I_stator
    //     Advantages: instantaneous, no noise amplification, no one-loop delay.
    //     Set m_accelerationPerAmp = kt / J  (mechanism units·s⁻² / A).
    //     Determine from SysId ka result: accelerationPerAmp ≈ 1 / (ka × J_effective).
    //
    //   TIME-BASED (fallback):
    //     α = Δv / Δt  — numerical differentiation of velocity.
    //     Noisy and delayed; only used when accelerationPerAmp is not supplied.
    //
    private final double m_accelerationPerAmp;   // 0 → time-based fallback
    private double m_lastTime;
    private double m_lastVelocity;

    /**
     * Time-based acceleration estimation (fallback).
     * Prefer the overload that takes {@code accelerationPerAmp} when kt/J is known.
     */
    public FlyWheel(Motor motor, double maxAcceleration, double maxJerk, Gains gains) {
        this(motor, maxAcceleration, maxJerk, gains, 0.0);
    }

    /**
     * Current-based acceleration estimation.
     *
     * @param accelerationPerAmp  kt / J — angular acceleration (mechanism units/s²) per amp of
     *                            stator current.  Find this via SysId: run the quasistatic routine,
     *                            then {@code accelerationPerAmp ≈ 1 / (ka × effective_inertia)}.
     *                            You can also compute it from motor specs:
     *                            {@code kt} (N·m/A) ÷ {@code J} (kg·m²) converted to your velocity units.
     */
    public FlyWheel(Motor motor, double maxAcceleration, double maxJerk, Gains gains, double accelerationPerAmp) {
        super(motor);
        m_gains              = gains;
        m_pid                = new PIDController(gains.kp, gains.ki, gains.kd);
        m_ff                 = new SimpleMotorFeedforward(gains.ks, gains.kv, gains.ka);
        m_profile            = new TrapezoidProfile(new TrapezoidProfile.Constraints(maxAcceleration, maxJerk));
        m_accelerationPerAmp = accelerationPerAmp;

        // Only needed for time-based fallback, but initialized regardless
        m_lastTime     = Timer.getFPGATimestamp();
        m_lastVelocity = motor.getMotorVelocity();
    }

    /**
     * Smooth velocity control using a pre-allocated TrapezoidProfile.
     * Call {@link #periodic()} every loop when using time-based mode.
     */
    public Command smartVelocityCommand(DoubleSupplier velocitySupplier, SubsystemBase... requirements) {
        return new RunCommand(() -> {
            TrapezoidProfile.State state = m_profile.calculate(
                    TimedRobot.kDefaultPeriod,
                    new TrapezoidProfile.State(m_motor.getMotorVelocity(), getAcceleration()),
                    new TrapezoidProfile.State(velocitySupplier.getAsDouble(), 0));

            double ff  = m_gains.ks * Math.signum(state.position)
                       + m_gains.kv * state.position
                       + m_gains.ka * state.velocity;
            double pid = m_pid.calculate(m_motor.getMotorVelocity(), state.position);
            setVoltage(pid + ff);
        }, requirements);
    }

    /** Direct velocity control without profiling. */
    public Command setDynamicVelocityCommand(DoubleSupplier velocity, SubsystemBase... requirements) {
        return new RunCommand(() -> setDynamicVelocity(velocity.getAsDouble()), requirements);
    }

    public void setDynamicVelocity(double velocity) {
        double ff  = m_ff.calculate(velocity);
        double pid = m_pid.calculate(m_motor.getMotorVelocity(), velocity);
        super.setVoltage(pid + ff);
    }

    /**
     * Update time-based acceleration state.
     * Only required when NOT using current-based mode (i.e., accelerationPerAmp was not set).
     * Call once per robot loop from the owning subsystem's periodic().
     */
    public void periodic() {
        m_lastTime     = Timer.getFPGATimestamp();
        m_lastVelocity = m_motor.getMotorVelocity();
    }

    public double getVelocity() { return m_motor.getMotorVelocity(); }

    /** Returns true if this instance uses current-based acceleration (preferred). */
    public boolean isCurrentBased() { return m_accelerationPerAmp > 0; }

    // ── Internal ─────────────────────────────────────────────────────────────

    private double getAcceleration() {
        if (m_accelerationPerAmp > 0) {
            // α = (kt/J) × I_stator — instantaneous, no noise amplification
            return m_accelerationPerAmp * m_motor.getMotorStatorCurrent();
        }
        // Fallback: numerical differentiation
        double dt = Timer.getFPGATimestamp() - m_lastTime;
        if (dt < 1e-6) return 0.0;
        return (m_motor.getMotorVelocity() - m_lastVelocity) / dt;
    }
}
