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
    private final TrapezoidProfile.Constraints m_constraints;

    private double m_lastTime;
    private double m_lastVelocity;

    public FlyWheel(Motor motor, double maxAcceleration, double maxJerk, Gains gains) {
        super(motor);
        m_gains       = gains;
        m_pid         = new PIDController(gains.kp, gains.ki, gains.kd);
        m_ff          = new SimpleMotorFeedforward(gains.ks, gains.kv, gains.ka);
        m_constraints = new TrapezoidProfile.Constraints(maxAcceleration, maxJerk);
        m_profile     = new TrapezoidProfile(m_constraints);

        // Initialize with current time so the first acceleration sample is valid
        m_lastTime     = Timer.getFPGATimestamp();
        m_lastVelocity = motor.getMotorVelocity();
    }

    /**
     * Smooth velocity control using a pre-allocated TrapezoidProfile (no per-loop object allocation).
     * Requires {@link #periodic()} to be called every loop from the subsystem.
     */
    public Command smartVelocityCommand(DoubleSupplier velocitySupplier, SubsystemBase... requirements) {
        return new RunCommand(() -> {
            TrapezoidProfile.State state = m_profile.calculate(
                    TimedRobot.kDefaultPeriod,
                    new TrapezoidProfile.State(m_motor.getMotorVelocity(), getAcceleration()),
                    new TrapezoidProfile.State(velocitySupplier.getAsDouble(), 0));

            // Manual FF to avoid deprecated calculate(v, a) overload
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

    /** Must be called once per robot loop (e.g., from the subsystem's periodic()). */
    public void periodic() {
        m_lastTime     = Timer.getFPGATimestamp();
        m_lastVelocity = m_motor.getMotorVelocity();
    }

    public double getVelocity() {
        return m_motor.getMotorVelocity();
    }

    private double getAcceleration() {
        double dt = Timer.getFPGATimestamp() - m_lastTime;
        if (dt < 1e-6) return 0.0;   // guard against divide-by-zero on first call
        return (m_motor.getMotorVelocity() - m_lastVelocity) / dt;
    }
}
