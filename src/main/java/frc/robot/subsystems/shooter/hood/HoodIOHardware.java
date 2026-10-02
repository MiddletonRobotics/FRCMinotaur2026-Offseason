package frc.robot.subsystems.shooter.hood;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.AudioConfigs;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.CoastOut;
import com.ctre.phoenix6.controls.PositionTorqueCurrentFOC;
import com.ctre.phoenix6.controls.StaticBrake;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;

import static frc.minolib.utilities.PhoenixUtility.tryUntilOk;

import frc.minolib.utilities.PhoenixUtility;
import frc.robot.constants.HoodConstants;

public class HoodIOHardware implements HoodIO {
    private final TalonFX motor;
    private final TalonFXConfiguration configuration;

    private final StatusSignal<Angle> position;
    private final StatusSignal<AngularVelocity> velocity;
    private final StatusSignal<Voltage> appliedVoltage;
    private final StatusSignal<Current> supplyCurrent;
    private final StatusSignal<Current> torqueCurrent;
    private final StatusSignal<Temperature> temperature;
    private final StatusSignal<Boolean> temperatureFault;

    private final VoltageOut voltageRequest = new VoltageOut(0.0)
        .withEnableFOC(true)
        .withUpdateFreqHz(0.0);

    private final PositionTorqueCurrentFOC positionRequest = new PositionTorqueCurrentFOC(0.0)
        .withUpdateFreqHz(0.0);

    private final CoastOut coastRequest = new CoastOut();
    private final StaticBrake brakeRequest = new StaticBrake();

    private double previousKP = 0.0;
    private double previousKD = 0.0;
    private double previousKS = 0.0;
    private double previousKV = 0.0;
    private double previousKA = 0.0;

    public HoodIOHardware() {
        motor = new TalonFX(HoodConstants.kKrakenX44.getDeviceID(), HoodConstants.kKrakenX44.getCANBus());
        configuration = new TalonFXConfiguration()
            .withMotorOutput(
                new MotorOutputConfigs()
                    .withInverted(HoodConstants.kMotorInverted ? InvertedValue.Clockwise_Positive : InvertedValue.CounterClockwise_Positive)
                    .withNeutralMode(NeutralModeValue.Brake)   
            ).withCurrentLimits(
                new CurrentLimitsConfigs()
                    .withStatorCurrentLimitEnable(true)
                    .withStatorCurrentLimit(HoodConstants.kMotorStatorCurrentLimit)
                    .withSupplyCurrentLimitEnable(true)
                    .withSupplyCurrentLimit(HoodConstants.kMotorSupplyCurrentLimit)
            ).withFeedback(
                new FeedbackConfigs()
                    .withSensorToMechanismRatio(HoodConstants.kMotorReduction)
            ).withAudio(
                new AudioConfigs()
                    .withBeepOnBoot(false)
                    .withBeepOnConfig(false)
            );

        tryUntilOk(5, () -> motor.getConfigurator().apply(configuration, 0.25));

        position = motor.getPosition();
        velocity = motor.getVelocity();
        appliedVoltage = motor.getMotorVoltage();
        supplyCurrent = motor.getSupplyCurrent();
        torqueCurrent = motor.getTorqueCurrent();
        temperature = motor.getDeviceTemp();
        temperatureFault = motor.getFault_DeviceTemp();

        tryUntilOk(5, () -> BaseStatusSignal.setUpdateFrequencyForAll(50.0, position, velocity, appliedVoltage, supplyCurrent, torqueCurrent, temperature, temperatureFault));
        tryUntilOk(5, () -> motor.optimizeBusUtilization(0.0, 0.25));

        PhoenixUtility.registerSignals(false, position, velocity, appliedVoltage, supplyCurrent, torqueCurrent, temperature, temperatureFault);
    }

    @Override
    public void updateInputs(HoodIOInputs inputs) {
        inputs.connected = BaseStatusSignal.isAllGood(position, velocity, appliedVoltage, supplyCurrent, torqueCurrent, temperature);
        inputs.positionRadians = Units.rotationsToRadians(position.getValueAsDouble());
        inputs.velocityRadiansPerSecond = Units.rotationsToRadians(position.getValueAsDouble());
        inputs.appliedVoltage = appliedVoltage.getValueAsDouble();
        inputs.supplyCurrentAmperes = supplyCurrent.getValueAsDouble();
        inputs.torqueCurrentAmperes = torqueCurrent.getValueAsDouble();
        inputs.temperatureCelsius = temperature.getValueAsDouble();
        inputs.temperatureFault = temperatureFault.getValue().booleanValue();
    }

    @Override
    public void applyOutputs(HoodIOOutputs outputs) {
        if (outputs.kP != previousKP || outputs.kD != previousKD || outputs.kS != previousKS || outputs.kV != previousKV || outputs.kA != previousKA) {
            configuration.Slot0 = new Slot0Configs()
                .withKP(outputs.kP)
                .withKD(outputs.kD)
                .withKS(outputs.kS)
                .withKV(outputs.kV)
                .withKA(outputs.kA);

            tryUntilOk(5, () -> motor.getConfigurator().apply(configuration.Slot0, 0.0));

            previousKP = outputs.kP;
            previousKD = outputs.kD;
            previousKS = outputs.kS;
            previousKV = outputs.kV;
            previousKA = outputs.kA;
        }

        switch (outputs.mode) {
            case BRAKE -> motor.setControl(brakeRequest);
            case COAST -> motor.setControl(coastRequest);
            case VOLTAGE_CONTROL -> motor.setControl(voltageRequest.withOutput(outputs.appliedVoltage));
            case CLOSED_LOOP -> motor.setControl(positionRequest.withPosition(Units.radiansToRotations(outputs.position)).withVelocity(Units.radiansToRotations(outputs.velocity)));
        }
    }

    @Override
    public void stop() {
        motor.stopMotor();
    }
}
