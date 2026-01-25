package frc.excalib.control.motor.controllers;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.units.measure.*;
import frc.excalib.control.motor.motor_specs.DirectionState;
import frc.excalib.control.motor.motor_specs.IdleState;

import java.util.ArrayList;
import java.util.HashMap;

import static com.ctre.phoenix6.signals.InvertedValue.Clockwise_Positive;
import static com.ctre.phoenix6.signals.InvertedValue.CounterClockwise_Positive;
import static frc.excalib.control.motor.motor_specs.DirectionState.FORWARD;

public class TalonFXMotor extends TalonFX implements Motor {
    private double POSITION_CONVERSION_FACTOR, VELOCITY_CONVERSION_FACTOR;
    private StatusSignal<Angle> poseSignal;
    private StatusSignal<AngularVelocity> velocitySignal;
    private StatusSignal<Current> currentSignal;
    private StatusSignal<Voltage> voltageSignal;
    private StatusSignal<Temperature> temperatureSignal;

    private CANBus canbus = new CANBus();
    private IdleState m_idleState = null;

    private final static ArrayList<TalonFXMotor> motors = new ArrayList<>();
    private final static HashMap<CANBus, ArrayList<BaseStatusSignal>> canMap = new HashMap<>();


    private void initMotor() {
        POSITION_CONVERSION_FACTOR = 1;
        VELOCITY_CONVERSION_FACTOR = 1;

        setIdleState(IdleState.BRAKE);

        poseSignal = super.getPosition();
        velocitySignal = super.getVelocity();
        currentSignal = super.getSupplyCurrent();
        voltageSignal = super.getMotorVoltage();
        temperatureSignal = super.getDeviceTemp();

        ArrayList<BaseStatusSignal> signals = canMap.get(this.canbus);

        if (signals == null) {
            canMap.put(this.canbus, new ArrayList<>());
            signals = canMap.get(this.canbus);
        }

        signals.add(poseSignal);
        signals.add(velocitySignal);
        signals.add(currentSignal);
        signals.add(voltageSignal);
        signals.add(temperatureSignal);

        motors.add(this);
    }

    public TalonFXMotor(int deviceId, CANBus canbus) {
        super(deviceId, canbus);
        this.canbus = canbus;
    }

    public TalonFXMotor(int canID) {
        this(canID, new CANBus());
    }

    public static void refreshAll() {
        for (TalonFXMotor motor : motors) motor.refresh();
        for (ArrayList<BaseStatusSignal> signals : canMap.values()) {
            BaseStatusSignal.refreshAll(signals.toArray(new BaseStatusSignal[0]));
        }
    }

    public void refresh() {
        ArrayList<BaseStatusSignal> signals = new ArrayList<>();
        signals.add(poseSignal);
        signals.add(velocitySignal);
        signals.add(currentSignal);
        signals.add(voltageSignal);
        signals.add(temperatureSignal);
        BaseStatusSignal.refreshAll(signals.toArray(new BaseStatusSignal[0]));
    }

    @Override
    public void setPercentage(double percentage) {
        super.setControl(
                new DutyCycleOut(percentage).withEnableFOC(true)
        );
    }

    @Override
    public void setFollower(int mainMotorID) {
        super.setControl(new Follower(mainMotorID, MotorAlignmentValue.Opposed));
    }

    @Override
    public void setIdleState(IdleState idleMode) {
        m_idleState = idleMode;
        super.setNeutralMode(idleMode == IdleState.BRAKE ? NeutralModeValue.Brake : NeutralModeValue.Coast);
    }

    @Override
    public double getMotorPosition() {
        return POSITION_CONVERSION_FACTOR * poseSignal.getValueAsDouble();
    }

    @Override
    public double getMotorVelocity() {
        return VELOCITY_CONVERSION_FACTOR * velocitySignal.getValueAsDouble();
    }

    @Override
    public double getCurrent() {
        return currentSignal.getValueAsDouble();
    }

    @Override
    public IdleState getIdleState() {
        return m_idleState;
    }

    @Override
    public double getVoltage() {
        return voltageSignal.getValueAsDouble();
    }

    @Override
    public double getTemperature() {
        return temperatureSignal.getValueAsDouble();
    }

    @Override
    public void setSoftLimit(DirectionState directionState, float limit) {
        var talonFXConfigurator = super.getConfigurator(); //TODO: implement
    }

    @Override
    public void setPositionConversionFactor(double conversionFactor) {
        POSITION_CONVERSION_FACTOR = conversionFactor;
    }

    @Override
    public void setVelocityConversionFactor(double conversionFactor) {
        VELOCITY_CONVERSION_FACTOR = conversionFactor;
    }

    @Override
    public void setCurrentLimit(int stallLimit, int freeLimit) {
        var talonFXConfigurator = super.getConfigurator();
        var limitConfigs = new CurrentLimitsConfigs();
        limitConfigs.SupplyCurrentLimitEnable = true;
        limitConfigs.SupplyCurrentLimit = freeLimit;

        talonFXConfigurator.apply(limitConfigs);
    }

    @Override
    public void setMotorPosition(double position) {
        super.setPosition(position / POSITION_CONVERSION_FACTOR);
    }

    @Override
    public void setInverted(DirectionState mode) {
        var talonFXConfigurator = new MotorOutputConfigs();
        talonFXConfigurator.withInverted(mode == FORWARD ? CounterClockwise_Positive : Clockwise_Positive);
        super.getConfigurator().apply(talonFXConfigurator);
    }

    @Override
    public void setMotorVoltage(double voltage) {
        super.setVoltage(voltage);
    }
}
