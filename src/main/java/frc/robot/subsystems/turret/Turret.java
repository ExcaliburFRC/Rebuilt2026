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
                        return Math.abs(
                                turretMechanism.getPosition().getRadians() -
                                        SOFT_LIMIT.limit(
                                                TURRET_CONTINUOUS_SOFTLIMIT.getSetpoint(
                                                        turretAngleSupplier.getAsDouble(),
                                                        turretRelativeAngleToTarget.getAsDouble()))) < PID_TOLERANCE;
                    }
                    return false;
                }
        );
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("Turret", inputs);
    }

    public Command defaultCommand() {
        Command c = new ConditionalCommand(
                Commands.none(),
                setPositionCommand(
                        () -> Rotation2d.fromRadians(
                                SOFT_LIMIT.limit(
                                        TURRET_CONTINUOUS_SOFTLIMIT.getSetpoint(
                                                turretAngleSupplier.getAsDouble(),
                                                turretRelativeAngleToTarget.getAsDouble())
                                )
                        )
                ),
                () -> turretTarget.equals(Target.IDLE)
        );
        c.addRequirements(this);
        return c;
    }

    public Command setPositionCommand(Supplier<Rotation2d> position) {
        return turretMechanism.setPositionCommand(position, this);
    }

    public Command setPositionFieldRelativeCommand(Supplier<Rotation2d> angle) {
        return setPositionCommand(
                () -> new Rotation2d(
                        (Math.PI - robotAngleSupplier.getAsDouble() + angle.get().getRadians())
                )
        );
    }

    public Command targetHubCommand() {
        return new InstantCommand(() -> turretTarget = Target.HUB, this);
    }

    public Command targetDeliveryCommand() {
        return new InstantCommand(() -> turretTarget = Target.DELIVERY, this);
    }

    public Command idleCommand() {
        return new InstantCommand(() -> turretTarget = Target.IDLE, this);
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
