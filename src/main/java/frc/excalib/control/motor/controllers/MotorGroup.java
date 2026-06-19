package frc.excalib.control.motor.controllers;

import frc.excalib.control.motor.motor_specs.DirectionState;
import frc.excalib.control.motor.motor_specs.IdleState;

public class MotorGroup implements Motor {

    private final Motor[] m_motors;

    public MotorGroup(Motor... motors) {
        this.m_motors = motors;
    }

    @Override
    public void stopMotor() {
        for (Motor m : m_motors) m.stopMotor();
    }

    @Override
    public void setMotorVoltage(double voltage) {
        for (Motor m : m_motors) m.setMotorVoltage(voltage);
    }

    @Override
    public void setPercentage(double percentage) {
        for (Motor m : m_motors) m.setPercentage(percentage);
    }

    @Override
    public void setFollower(int mainMotorID) {
        for (Motor m : m_motors) {
            if (m.getDeviceID() != mainMotorID) m.setFollower(mainMotorID);
        }
    }

    @Override
    public void setIdleState(IdleState idleMode) {
        for (Motor m : m_motors) m.setIdleState(idleMode);
    }

    @Override
    public void setSoftLimit(DirectionState directionState, float limit) {
        for (Motor m : m_motors) m.setSoftLimit(directionState, limit);
    }

    @Override
    public void setInverted(DirectionState mode) {
        for (Motor m : m_motors) m.setInverted(mode);
    }

    @Override
    public void setPositionConversionFactor(double conversionFactor) {
        for (Motor m : m_motors) m.setPositionConversionFactor(conversionFactor);
    }

    @Override
    public void setVelocityConversionFactor(double conversionFactor) {
        for (Motor m : m_motors) m.setVelocityConversionFactor(conversionFactor);
    }

    @Override
    public void setCurrentLimit(int stator, int supply) {
        for (Motor m : m_motors) m.setCurrentLimit(stator, supply);
    }

    @Override
    public void setMotorPosition(double position) {
        for (Motor m : m_motors) m.setMotorPosition(position);
    }

    @Override public IdleState getIdleState()  { return m_motors[0].getIdleState(); }
    @Override public int       getDeviceID()   { return m_motors.length > 0 ? m_motors[0].getDeviceID() : -1; }

    @Override
    public double getMotorPosition() {
        double sum = 0;
        for (Motor m : m_motors) sum += m.getMotorPosition();
        return sum / m_motors.length;
    }

    @Override
    public double getMotorVelocity() {
        double sum = 0;
        for (Motor m : m_motors) sum += m.getMotorVelocity();
        return sum / m_motors.length;
    }

    @Override
    public double getCurrent() {
        double sum = 0;
        for (Motor m : m_motors) sum += m.getCurrent();
        return sum / m_motors.length;
    }

    @Override
    public double getMotorStatorCurrent() {
        double sum = 0;
        for (Motor m : m_motors) sum += m.getMotorStatorCurrent();
        return sum / m_motors.length;
    }

    @Override
    public double getVoltage() {
        double sum = 0;
        for (Motor m : m_motors) sum += m.getVoltage();
        return sum / m_motors.length;
    }

    @Override
    public double getTemperature() {
        double max = Double.MIN_VALUE;
        for (Motor m : m_motors) max = Math.max(max, m.getTemperature());
        return max;
    }
}
