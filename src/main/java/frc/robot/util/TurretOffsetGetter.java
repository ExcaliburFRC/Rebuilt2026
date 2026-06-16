package frc.robot.util;

import edu.wpi.first.math.geometry.Rotation2d;

import java.util.Date;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

public class TurretOffsetGetter {
    public static final TurretOffsetGetter instance = new TurretOffsetGetter();
    private Supplier<Rotation2d> turretOffsetSupplier;
    private DoubleSupplier robotRotationalVelSup = () -> 0;
    private DoubleSupplier turretRotationalVelSup = () -> 0;

    private TurretOffsetGetter() {
        this.turretOffsetSupplier = Rotation2d::new;
    }

    ;

    public void setTurretOffsetSupplier(Supplier<Rotation2d> turretOffsetSupplier) {
        this.turretOffsetSupplier = turretOffsetSupplier;
    }

    public Rotation2d getTurretOffset() {
        return this.turretOffsetSupplier.get();
    }

    public boolean isFast() {
        return this.robotRotationalVelSup.getAsDouble() + this.turretRotationalVelSup.getAsDouble() > Math.PI;
    }

    public void setRobotRotationalVel(DoubleSupplier robotRotationalVel) {
        this.robotRotationalVelSup = robotRotationalVel;
    }

    public void setTurretRotationalVelSup(DoubleSupplier turretRotationalVelSup) {
        this.turretOffsetSupplier = turretOffsetSupplier;
    }

}
