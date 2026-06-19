package frc.excalib.mechanisms.turret;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.excalib.control.gains.Gains;
import frc.excalib.control.limits.ContinuousSoftLimit;
import frc.excalib.control.motor.controllers.Motor;
import frc.excalib.mechanisms.Mechanism;
import monologue.Annotations.Log;
import monologue.Logged;

import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

public final class Turret extends Mechanism implements Logged {

    private final ContinuousSoftLimit    m_rotationLimit;
    public  final PIDController          m_anglePIDcontroller;
    private final SimpleMotorFeedforward m_angleFFcontroller;
    private final DoubleSupplier         m_POSITION_SUPPLIER;

    // Optional trapezoid profile (null when not used)
    private final TrapezoidProfile       m_profile;
    private TrapezoidProfile.State       m_profileState = new TrapezoidProfile.State(0, 0);

    private double smartSetpoint  = 0;
    private double wantedSetpoint = 0;

    /** Basic constructor — no motion profiling. */
    public Turret(Motor motor,
                  ContinuousSoftLimit rotationLimit,
                  Gains angleGains,
                  double PIDtolerance,
                  DoubleSupplier positionSupplier) {
        this(motor, rotationLimit, angleGains, PIDtolerance, positionSupplier, null);
    }

    /**
     * Constructor with TrapezoidProfile constraints.
     * When constraints are provided, setPosition() uses a trapezoid profile to generate
     * smooth velocity setpoints, and applies kv feedforward from the profile's velocity.
     */
    public Turret(Motor motor,
                  ContinuousSoftLimit rotationLimit,
                  Gains angleGains,
                  double PIDtolerance,
                  DoubleSupplier positionSupplier,
                  TrapezoidProfile.Constraints constraints) {
        super(motor);
        m_rotationLimit      = rotationLimit;
        m_anglePIDcontroller = new PIDController(angleGains.kp, angleGains.ki, angleGains.kd);
        m_angleFFcontroller  = new SimpleMotorFeedforward(angleGains.ks, angleGains.kv, angleGains.ka);
        m_anglePIDcontroller.setTolerance(PIDtolerance);
        m_POSITION_SUPPLIER  = positionSupplier;
        m_profile            = (constraints != null) ? new TrapezoidProfile(constraints) : null;
    }

    public Command setPositionCommand(Supplier<Rotation2d> wantedPosition, SubsystemBase... requirements) {
        return new RunCommand(() -> setPosition(wantedPosition.get()), requirements);
    }

    /**
     * Drive turret to position.
     *
     * Control law:
     *   - ContinuousSoftLimit wrapping for shortest valid path
     *   - ks: static friction in the direction of the error (not PID sign)
     *   - kv: velocity feedforward from profile velocity (or motor velocity if unprofiled)
     *   - PID: position feedback
     */
    public void setPosition(Rotation2d wantedPosition) {
        this.wantedSetpoint = wantedPosition.getRadians();

        // getSetpoint() already returns a limit-compliant wrapped angle — no redundant limit() needed
        this.smartSetpoint = m_rotationLimit.getSetpoint(
                getPosition().getRadians(), wantedPosition.getRadians());

        double error     = this.smartSetpoint - m_POSITION_SUPPLIER.getAsDouble();
        double kvVelocity;
        double pidSetpoint;

        if (m_profile != null) {
            m_profileState = m_profile.calculate(
                    TimedRobot.kDefaultPeriod,
                    m_profileState,
                    new TrapezoidProfile.State(this.smartSetpoint, 0));
            pidSetpoint = m_profileState.position;
            kvVelocity  = m_profileState.velocity;
        } else {
            pidSetpoint = this.smartSetpoint;
            kvVelocity  = m_motor.getMotorVelocity();
        }

        double pid = m_anglePIDcontroller.calculate(m_POSITION_SUPPLIER.getAsDouble(), pidSetpoint);
        double ks  = m_anglePIDcontroller.atSetpoint() ? 0 : m_angleFFcontroller.getKs() * Math.signum(error);
        double kv  = m_angleFFcontroller.getKv() * kvVelocity;

        super.setVoltage(pid + ks + kv);
    }

    public Rotation2d getPosition() {
        return new Rotation2d(m_POSITION_SUPPLIER.getAsDouble());
    }

    public Command stopTurret(SubsystemBase... requirements) {
        return new InstantCommand(super.m_motor::stopMotor, requirements);
    }

    @Log.NT public double getSmartSetpoint()  { return smartSetpoint;  }
    @Log.NT public double getWantedSetpoint() { return wantedSetpoint; }
}
