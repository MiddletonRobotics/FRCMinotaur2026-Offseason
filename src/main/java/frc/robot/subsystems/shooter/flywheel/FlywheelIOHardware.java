package frc.robot.subsystems.shooter.flywheel;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.AudioConfigs;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.CoastOut;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.StaticBrake;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularAcceleration;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;

import static frc.minolib.utilities.PhoenixUtility.tryUntilOk;

import frc.minolib.utilities.PhoenixUtility;
import frc.robot.constants.FlywheelConstants;

public class FlywheelIOHardware implements FlywheelIO {
    private TalonFX primaryMotor;
    private TalonFX followerOneMotor;
    private TalonFX followerTwoMotor;
    private TalonFX followerThreeMotor;

    private TalonFXConfiguration configuration;

    private final StatusSignal<Angle> primaryPosition;
    private final StatusSignal<AngularVelocity> primaryVelocity;
    private final StatusSignal<AngularAcceleration> primaryAcceleration;
    private final StatusSignal<Voltage> primaryAppliedVoltage;
    private final StatusSignal<Current> primaryTorqueCurrent;

    private final StatusSignal<Current> primarySupplyCurrent;
    private final StatusSignal<Current> followerOneSupplyCurrent;
    private final StatusSignal<Current> followerTwoSupplyCurrent;
    private final StatusSignal<Current> followerThreeSupplyCurrent;

    private final StatusSignal<Temperature> primaryTemperature;
    private final StatusSignal<Temperature> followerOneTemperature;
    private final StatusSignal<Temperature> followerTwoTemperature;
    private final StatusSignal<Temperature> followerThreeTemperature;

    private final VoltageOut voltageRequest = new VoltageOut(0.0)
        .withEnableFOC(true)
        .withUpdateFreqHz(0.0);

    private final VelocityVoltage velocityRequest = new VelocityVoltage(0.0)
        .withEnableFOC(true)
        .withUpdateFreqHz(0.0);

    private final CoastOut coastRequest = new CoastOut();
    private final StaticBrake brakeRequest = new StaticBrake();

    private double previousKP = 0.0;
    private double previousKD = 0.0;
    private double previousKS = 0.0;
    private double previousKV = 0.0;
    private double previousKA = 0.0;

    public FlywheelIOHardware() {
        primaryMotor = new TalonFX(FlywheelConstants.kPrimaryMotor.getDeviceID(), FlywheelConstants.kPrimaryMotor.getCANBus());
        followerOneMotor = new TalonFX(FlywheelConstants.kFollowerOneMotor.getDeviceID(), FlywheelConstants.kFollowerOneMotor.getCANBus());
        followerTwoMotor = new TalonFX(FlywheelConstants.kFollowerTwoMotor.getDeviceID(), FlywheelConstants.kFollowerThreeMotor.getCANBus());
        followerThreeMotor = new TalonFX(FlywheelConstants.kFollowerThreeMotor.getDeviceID(), FlywheelConstants.kFollowerThreeMotor.getCANBus());

        configuration = new TalonFXConfiguration()
            .withCurrentLimits(
                new CurrentLimitsConfigs()
                    .withStatorCurrentLimitEnable(true)
                    .withStatorCurrentLimit(FlywheelConstants.kMotorStatorCurrentLimit)
                    .withSupplyCurrentLimitEnable(true)
                    .withSupplyCurrentLimit(FlywheelConstants.kMotorSupplyCurrentLimit)
            );

        tryUntilOk(5, () -> followerOneMotor.getConfigurator().apply(configuration, 0.25));
        tryUntilOk(5, () -> followerTwoMotor.getConfigurator().apply(configuration, 0.25));
        tryUntilOk(5, () -> followerThreeMotor.getConfigurator().apply(configuration, 0.25));

        followerOneMotor.setControl(new Follower(primaryMotor.getDeviceID(), MotorAlignmentValue.Aligned));
        followerTwoMotor.setControl(new Follower(primaryMotor.getDeviceID(), MotorAlignmentValue.Opposed));
        followerThreeMotor.setControl(new Follower(primaryMotor.getDeviceID(), MotorAlignmentValue.Opposed));

        configuration
            .withMotorOutput(
                new MotorOutputConfigs()
                    .withInverted(FlywheelConstants.kPrimaryMotorInverted ? InvertedValue.Clockwise_Positive : InvertedValue.CounterClockwise_Positive)
                    .withNeutralMode(NeutralModeValue.Brake)
            ).withFeedback(
                new FeedbackConfigs()
                    .withSensorToMechanismRatio(FlywheelConstants.kMotorReduction)
            ).withAudio(
                new AudioConfigs()
                    .withBeepOnBoot(false)
                    .withBeepOnConfig(false)
            );

        tryUntilOk(5, () -> primaryMotor.getConfigurator().apply(configuration, 0.25));

        primaryPosition = primaryMotor.getPosition();
        primaryVelocity = primaryMotor.getVelocity();
        primaryAcceleration = primaryMotor.getAcceleration();
        primaryAppliedVoltage = primaryMotor.getMotorVoltage();
        primaryTorqueCurrent = primaryMotor.getTorqueCurrent();

        primarySupplyCurrent = primaryMotor.getSupplyCurrent();
        followerOneSupplyCurrent = followerOneMotor.getSupplyCurrent();
        followerTwoSupplyCurrent = followerTwoMotor.getSupplyCurrent();
        followerThreeSupplyCurrent = followerThreeMotor.getSupplyCurrent();

        primaryTemperature = primaryMotor.getDeviceTemp();
        followerOneTemperature = followerOneMotor.getDeviceTemp();
        followerTwoTemperature = followerTwoMotor.getDeviceTemp();
        followerThreeTemperature = followerThreeMotor.getDeviceTemp();

        tryUntilOk(5, () -> BaseStatusSignal.setUpdateFrequencyForAll(
            50.0, 
            primaryPosition,
            primaryVelocity,
            primaryAcceleration,
            primaryAppliedVoltage,
            primaryTorqueCurrent,
            primarySupplyCurrent,
            followerOneSupplyCurrent,
            followerTwoSupplyCurrent,
            followerThreeSupplyCurrent,
            primaryTemperature,
            followerOneTemperature,
            followerTwoTemperature,
            followerThreeTemperature
        ));

        tryUntilOk(5, ()-> primaryMotor.optimizeBusUtilization(0.0, 0.25));
        tryUntilOk(5, () -> followerOneMotor.optimizeBusUtilization(0.0, 0.25));
        tryUntilOk(5, () -> followerTwoMotor.optimizeBusUtilization(0.0, 0.25));
        tryUntilOk(5, () -> followerThreeMotor.optimizeBusUtilization(0.0, 0.25));

        PhoenixUtility.registerSignals(
            false,
            primaryPosition,
            primaryVelocity,
            primaryAcceleration,
            primaryAppliedVoltage,
            primaryTorqueCurrent,
            primarySupplyCurrent,
            followerOneSupplyCurrent,
            followerTwoSupplyCurrent,
            followerThreeSupplyCurrent,
            primaryTemperature,
            followerOneTemperature,
            followerTwoTemperature,
            followerThreeTemperature
        );
    }

    @Override
    public void updateInputs(FlywheelIOInputs inputs) {
        inputs.primaryConnected = BaseStatusSignal.isAllGood(primaryPosition, primaryVelocity, primaryAcceleration, primaryAppliedVoltage, primarySupplyCurrent, primaryTorqueCurrent);
        inputs.followerOneConnected = BaseStatusSignal.isAllGood(followerOneSupplyCurrent);
        inputs.followerTwoConnected = BaseStatusSignal.isAllGood(followerTwoSupplyCurrent);
        inputs.followerThreeConnected = BaseStatusSignal.isAllGood(followerThreeSupplyCurrent);

        inputs.position = Units.rotationsToRadians(primaryPosition.getValueAsDouble());
        inputs.velocity = Units.rotationsToRadians(primaryVelocity.getValueAsDouble());
        inputs.acceleration = Units.rotationsToRadians(primaryAcceleration.getValueAsDouble());
        inputs.appliedVoltage = primaryAppliedVoltage.getValueAsDouble();
        inputs.torqueCurrent = primaryTorqueCurrent.getValueAsDouble();

        inputs.primarySupplyCurrent = primarySupplyCurrent.getValueAsDouble();
        inputs.followerOneSupplyCurrent = followerOneSupplyCurrent.getValueAsDouble();
        inputs.followerTwoSupplyCurrent = followerTwoSupplyCurrent.getValueAsDouble();
        inputs.followerThreeSupplyCurrent = followerThreeSupplyCurrent.getValueAsDouble();

        inputs.primaryTemperature = primaryTemperature.getValueAsDouble();
        inputs.followerOneTemperature = followerOneTemperature.getValueAsDouble();
        inputs.followerTwoTemperature = followerTwoTemperature.getValueAsDouble();
        inputs.followerThreeTemperature = followerThreeTemperature.getValueAsDouble();
    }

    @Override
    public void applyOutputs(FlywheelIOOutputs outputs) {
        if (previousKP != outputs.kP || previousKD != outputs.kD || previousKS != outputs.kS || previousKV != outputs.kV || previousKA != outputs.kA) {
            configuration.withSlot0(
                new Slot0Configs()
                    .withKP(outputs.kP)
                    .withKD(outputs.kD)
                    .withKS(outputs.kS)
                    .withKV(outputs.kV)
                    .withKA(outputs.kA)
            );

            tryUntilOk(5, () -> primaryMotor.getConfigurator().apply(configuration.Slot0, 0.0));

            previousKP = outputs.kP;
            previousKD = outputs.kD;
            previousKS = outputs.kS;
            previousKV = outputs.kV;
            previousKA = outputs.kA;
        }

        switch (outputs.mode) {
            case BRAKE -> primaryMotor.setControl(brakeRequest);
            case COAST -> primaryMotor.setControl(coastRequest);
            case VOLTAGE_CONTROL -> primaryMotor.setControl(voltageRequest.withOutput(outputs.appliedVoltage));
            case VELOCITY_CONTROL -> primaryMotor.setControl(velocityRequest.withVelocity(outputs.goalVelocity));
        }
    }

    @Override
    public void stop() {
        primaryMotor.stopMotor();
    }
}