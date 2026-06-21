package frc.excalib.mechanisms.Arm;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.excalib.control.gains.Gains;
import frc.excalib.control.limits.SoftLimit;
import frc.excalib.control.math.physics.Mass;
import frc.excalib.control.motor.controllers.Motor;
import frc.excalib.mechanisms.Mechanism;

import java.util.function.Consumer;
import java.util.function.DoubleSupplier;

public class Arm extends Mechanism {

    private final Mass         m_mass;
    private final PIDController m_pid;
    public  final DoubleSupplier ANGLE_SUPPLIER;
    public  final SoftLimit      velocityLimit;
    public  final double         m_ks, m_kg, m_kv;
    private final double         m_maxOutputVoltage;

    /**
     * @param maxOutputVoltage  peak voltage the arm is allowed to output (V).
     *                          Use this to limit arm speed independent of the velocity soft-limit.
     *                          Typical safe starting value: 4–6 V. Max is 12 V.
     */
    public Arm(Motor motor,
               DoubleSupplier angleSupplier,
               SoftLimit velocityLimit,
               Gains gains,
               Mass mass,
               double maxOutputVoltage) {
        super(motor);
        ANGLE_SUPPLIER     = angleSupplier;
        this.velocityLimit = velocityLimit;
        m_ks               = gains.ks;
        m_kg               = gains.kg;
        m_kv               = gains.kv;
        m_pid              = new PIDController(gains.kp, gains.ki, gains.kd);
        m_mass             = mass;
        m_maxOutputVoltage = Math.abs(maxOutputVoltage);
    }

    /** Backwards-compatible constructor — defaults to 12 V max output. */
    public Arm(Motor motor,
               DoubleSupplier angleSupplier,
               SoftLimit velocityLimit,
               Gains gains,
               Mass mass) {
        this(motor, angleSupplier, velocityLimit, gains, mass, 12.0);
    }

    /**
     * Moves the arm to a dynamic angle setpoint (radians).
     *
     * Control law:
     *   1. Compute desired velocity from position error over one loop period.
     *   2. Clamp velocity to the configured soft limit.
     *   3. Apply static friction (ks), velocity (kv), and gravity (kg) feedforward.
     *   4. Add PID correction.
     *   5. Clamp total output to ±12 V.
     */
    public Command anglePositionControlCommand(
            DoubleSupplier setpointSupplier,
            Consumer<Boolean> toleranceConsumer,
            double maxOffset,
            SubsystemBase... requirements) {

        return new RunCommand(() -> {
            double setpoint = setpointSupplier.getAsDouble();
            double angle    = ANGLE_SUPPLIER.getAsDouble();
            double error    = setpoint - angle;

            double velocitySetpoint = velocityLimit.limit(error / TimedRobot.kDefaultPeriod);

            double phyOutput = m_ks * Math.signum(velocitySetpoint)
                             + m_kv * velocitySetpoint
                             + m_kg * m_mass.getCenterOfMass().getX();

            double pid    = m_pid.calculate(angle, setpoint);
            double output = MathUtil.clamp(phyOutput + pid, -m_maxOutputVoltage, m_maxOutputVoltage);

            super.setVoltage(output);
            toleranceConsumer.accept(Math.abs(error) <= maxOffset);
        }, requirements);
    }

    public Command goToAngleCommand(double angle, Consumer<Boolean> toleranceConsumer, double maxOffset, SubsystemBase... requirements) {
        return anglePositionControlCommand(() -> angle, toleranceConsumer, maxOffset, requirements);
    }
}
