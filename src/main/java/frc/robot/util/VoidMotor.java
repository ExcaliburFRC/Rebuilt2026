package frc.robot.util;

import frc.excalib.control.motor.controllers.Motor;
import frc.excalib.control.motor.motor_specs.DirectionState;
import frc.excalib.control.motor.motor_specs.IdleState;

/**
 * A no-op Motor implementation used during simulation or replay.
 * All read methods return zero; all write methods are ignored.
 */
public class VoidMotor implements Motor {

    @Override
    public void stopMotor() {}

    @Override
    public void setMotorVoltage(double voltage) {}

    @Override
    public void setPercentage(double percentage) {}

    @Override
    public void setFollower(int mainMotorID) {}

    @Override
    public void setIdleState(IdleState idleMode) {}

    @Override
    public IdleState getIdleState() {
        return IdleState.COAST;
    }

    @Override
    public int getDeviceID() {
        return -1;
    }

    @Override
    public double getMotorPosition() {
        return 0.0;
    }

    @Override
    public double getMotorVelocity() {
        return 0.0;
    }

    @Override
    public double getCurrent() {
        return 0.0;
    }

    @Override
    public double getVoltage() {
        return 0.0;
    }

    @Override
    public double getTemperature() {
        return 0.0;
    }

    @Override
    public void setSoftLimit(DirectionState directionState, float limit) {}

    @Override
    public void setInverted(DirectionState mode) {}

    @Override
    public void setPositionConversionFactor(double conversionFactor) {}

    @Override
    public void setVelocityConversionFactor(double conversionFactor) {}

    @Override
    public void setCurrentLimit(int stator, int supply) {}

    @Override
    public void setMotorPosition(double position) {}
}
