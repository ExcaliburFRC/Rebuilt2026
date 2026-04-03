package frc.robot.subsystems.turret;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.*;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.excalib.control.motor.controllers.Motor;
import frc.robot.util.Target;
import monologue.Annotations.Log;
import monologue.Logged;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import static frc.robot.subsystems.turret.TurretConstants.*;

public class Turret extends SubsystemBase implements Logged {

    private final TurretIO io;
    private final TurretIOInputsAutoLogged inputs = new TurretIOInputsAutoLogged();

    @Log.NT
    private double currentSetpoint = 0.0;
    @Log.NT
    private double currentWantedSetpoint = 0.0;

    public final frc.excalib.mechanisms.turret.Turret turretMechanism;

    /** Reads encoder position from logged inputs for correct replay behaviour. */
    public final DoubleSupplier turretAngleSupplier;

    public final DoubleSupplier turretRelativeAngleToTarget;
    public final DoubleSupplier robotAngleSupplier;

    public final Trigger isTurretAlligned;
    public Target turretTarget;

    public Turret(TurretIO io, DoubleSupplier turretRelativeAngleToTarget, DoubleSupplier robotAngleSupplier) {
        this.io = io;
        this.turretRelativeAngleToTarget = turretRelativeAngleToTarget;
        this.robotAngleSupplier = robotAngleSupplier;
        turretTarget = Target.HUB;

        // turretAngleSupplier reads from logged inputs so replay works correctly
        turretAngleSupplier = () -> inputs.encoderPositionRotations * ENCODER_POSITION_CONVERSION_FACTOR;

        Motor turretMotor = io.getTurretMotor();

        turretMechanism = new frc.excalib.mechanisms.turret.Turret(
                turretMotor,
                TURRET_CONTINUOUS_SOFTLIMIT,
                TURRET_GAINS,
                PID_TOLERANCE,
                this::getEncoderPosition,
                new TrapezoidProfile.Constraints(Math.PI * 60, Math.PI * 100)
        );

        isTurretAlligned = new Trigger(
                () -> {
                    if (turretTarget.equals(Target.IDLE)) {
                        return true;
                    } else if (turretTarget.equals(Target.HUB) || turretTarget.equals(Target.DELIVERY)) {
                        double current = turretMechanism.getPosition().getRadians();
                        double target = SOFT_LIMIT.limit(
                                TURRET_CONTINUOUS_SOFTLIMIT.getSetpoint(
                                        turretAngleSupplier.getAsDouble(),
                                        turretRelativeAngleToTarget.getAsDouble()));
                        return edu.wpi.first.math.MathUtil.isNear(target, current, PID_TOLERANCE);
                    }
                    return false;
                }
        );
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("Turret", inputs);

        if (turretTarget.equals(Target.IDLE)) {
            turretMechanism.setVoltage(0);
        } else if (!turretTarget.equals(Target.MANUAL)) {
            double wanted = (turretTarget == Target.HUB || turretTarget == Target.DELIVERY) ? 
                            turretRelativeAngleToTarget.getAsDouble() : 0.0;
            currentWantedSetpoint = wanted;
            double smart = TURRET_CONTINUOUS_SOFTLIMIT.getSetpoint(turretAngleSupplier.getAsDouble(), wanted);
            currentSetpoint = SOFT_LIMIT.limit(smart);
            turretMechanism.setPosition(Rotation2d.fromRadians(currentSetpoint));
        }
    }

    public void setTargetAngle(double angleRadians) {
        this.turretTarget = Target.MANUAL;
        this.currentSetpoint = SOFT_LIMIT.limit(angleRadians);
        turretMechanism.setPosition(Rotation2d.fromRadians(currentSetpoint));
    }

    public Command defaultCommand() {
        return Commands.idle(this).withName("TurretDefaultCommand");
    }

    public Command setPositionCommand(Supplier<Rotation2d> position) {
        return Commands.sequence(
                Commands.runOnce(() -> turretTarget = Target.MANUAL),
                turretMechanism.setPositionCommand(position, this)
        ).withName("TurretManualSetPosition");
    }

    public Command setPositionFieldRelativeCommand(Supplier<Rotation2d> angle) {
        return setPositionCommand(
                () -> new Rotation2d(
                        (Math.PI - robotAngleSupplier.getAsDouble() + angle.get().getRadians())
                )
        );
    }

    public Command targetHubCommand() {
        return Commands.runOnce(() -> turretTarget = Target.HUB, this);
    }

    public Command targetDeliveryCommand() {
        return Commands.runOnce(() -> turretTarget = Target.DELIVERY, this);
    }

    public Command idleCommand() {
        return Commands.runOnce(() -> turretTarget = Target.IDLE, this);
    }

    @AutoLogOutput(key = "Turret/EncoderPositionRad")
    @Log.NT
    public double getEncoderPosition() {
        return turretAngleSupplier.getAsDouble();
    }

    @AutoLogOutput(key = "Turret/Target")
    @Log.NT
    public String getTarget() {
        return turretTarget.name();
    }

    @AutoLogOutput(key = "Turret/OnFieldAngleDeg")
    @Log.NT
    public double getTurretOnFieldAngle() {
        return Units.radiansToDegrees(robotAngleSupplier.getAsDouble() + (turretMechanism.getPosition().getRadians()) - Math.PI);
    }

    @AutoLogOutput(key = "Turret/IsAligned")
    @Log.NT
    public boolean isTurretAlligned() {
        return isTurretAlligned.getAsBoolean();
    }

}
