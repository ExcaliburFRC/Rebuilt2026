package frc.excalib.mechanisms.linear_extension;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.excalib.control.gains.Gains;
import frc.excalib.control.motor.controllers.Motor;
import frc.excalib.mechanisms.Mechanism;

import java.util.function.DoubleSupplier;

public class LinearExtension extends Mechanism {

    private final DoubleSupplier    m_positionSupplier;
    private final DoubleSupplier    m_angleSupplier;
    private final PIDController     m_pid;
    private final double            m_tolerance;
    private final Gains             m_gains;
    private final TrapezoidProfile  m_profile;

    public LinearExtension(Motor motor,
                           DoubleSupplier positionSupplier,
                           DoubleSupplier angleSupplier,
                           Gains gains,
                           TrapezoidProfile.Constraints constraints,
                           double tolerance) {
        super(motor);
        m_positionSupplier = positionSupplier;
        m_angleSupplier    = angleSupplier;
        m_gains            = gains;
        m_pid              = new PIDController(gains.kp, gains.ki, gains.kd);
        m_profile          = new TrapezoidProfile(constraints);   // pre-allocated, not per-loop
        m_tolerance        = tolerance;
    }

    public Command extendCommand(DoubleSupplier lengthSetPoint, SubsystemBase... requirements) {
        return new RunCommand(() -> {
            TrapezoidProfile.State state = m_profile.calculate(
                    TimedRobot.kDefaultPeriod,
                    new TrapezoidProfile.State(m_positionSupplier.getAsDouble(), m_motor.getMotorVelocity()),
                    new TrapezoidProfile.State(lengthSetPoint.getAsDouble(), 0));

            double gravity = m_gains.kg * Math.sin(m_angleSupplier.getAsDouble());
            boolean atTarget = Math.abs(m_positionSupplier.getAsDouble() - lengthSetPoint.getAsDouble()) <= m_tolerance;

            double ff = atTarget
                    ? gravity
                    : m_gains.ks * Math.signum(state.velocity) + m_gains.kv * state.velocity + gravity;

            setVoltage(m_pid.calculate(m_positionSupplier.getAsDouble(), state.position) + ff);
        }, requirements);
    }

    public double logVoltage()   { return m_motor.getVoltage(); }
    public double logVelocity()  { return m_motor.getMotorVelocity(); }
    public double logPosition()  { return m_positionSupplier.getAsDouble(); }
}
