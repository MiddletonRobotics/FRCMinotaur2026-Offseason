package frc.robot.subsystems.shooter.hood;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.minolib.advantagekit.LoggedTracer;
import frc.minolib.advantagekit.LoggedTunableNumber;
import frc.robot.Robot;
import frc.robot.constants.HoodConstants;
import frc.robot.subsystems.shooter.hood.HoodIO.HoodIOOutputMode;
import frc.robot.subsystems.shooter.hood.HoodIO.HoodIOOutputs;

import lombok.Getter;
import lombok.Setter;

public class Hood extends SubsystemBase {
    private static final LoggedTunableNumber kTolerenceDegrees = new LoggedTunableNumber("Hood/ToleranceDegrees" ,1.0);
    private static final LoggedTunableNumber kHomingVoltage = new LoggedTunableNumber("Hood/HomingVoltage", -3);
    private static final LoggedTunableNumber kHomingVelocityThreshold = new LoggedTunableNumber("Hood/HomingVelocityThreshold", 0.05);

    private final HoodIO io;
    private final HoodIOInputsAutoLogged inputs = new HoodIOInputsAutoLogged();
    private final HoodIOOutputs outputs = new HoodIOOutputs();

    private final Debouncer motorConnectedDebouncer = new Debouncer(0.5, Debouncer.DebounceType.kFalling);
    private final Alert motorDisconnectedAlert = new Alert("Hood motor disconnected!", Alert.AlertType.kError);

    @Setter private BooleanSupplier coastOverride = () -> false;

    private double goalAngle = 0.0;
    private double goalVelocity = 0.0;

    private static double hoodOffset = 0.0;
    @Getter private boolean zeroed = false;

    public Hood(HoodIO io) {
        this.io = io;
    }

    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("Hood", inputs);

        motorDisconnectedAlert.set(
        Robot.showHardwareAlerts() && !motorConnectedDebouncer.calculate(inputs.connected));

        Robot.batteryLogger.reportCurrentUsage("Hood", inputs.connected ? inputs.supplyCurrentAmperes : 0.0);

        if (DriverStation.isDisabled() || (!zeroed && outputs.mode != HoodIOOutputMode.VOLTAGE_CONTROL)) {
            outputs.mode = HoodIOOutputMode.BRAKE;

            if (coastOverride.getAsBoolean()) {
                outputs.mode = HoodIOOutputMode.COAST;
            }
        }

        outputs.kP = HoodConstants.kP.get();
        outputs.kD = HoodConstants.kD.get();

        outputs.position = MathUtil.clamp(goalAngle, HoodConstants.kMinimumPosition, HoodConstants.kMaximumPosition) - hoodOffset;
        outputs.velocity = goalVelocity;
        outputs.mode = HoodIOOutputMode.CLOSED_LOOP;

        io.applyOutputs(outputs);
        LoggedTracer.record("HoodPeriodic");
    }

    private void setGoalParameters(double angle, double velocity) {
        this.goalAngle = angle;
        this.goalVelocity = velocity;
    }

    public double getMeasuredAngleRadians() {
        return inputs.positionRadians + hoodOffset;
    }

    public boolean atSetpoint() {
        return DriverStation.isEnabled() && zeroed && Math.abs(getMeasuredAngleRadians() - goalAngle) <= Units.degreesToRadians(kTolerenceDegrees.get());
    }

    public Command zeroCommand() {
        return run(() -> {
            outputs.appliedVoltage = kHomingVoltage.get();
            outputs.mode = HoodIOOutputMode.VOLTAGE_CONTROL;
            zeroed = false;
        }).raceWith(
            Commands.waitSeconds(0.5)
                .andThen(Commands.waitUntil(() ->Math.abs(inputs.velocityRadiansPerSecond) <= kHomingVelocityThreshold.get()))
        ).andThen(() -> {
            hoodOffset = HoodConstants.kHomedPosition - inputs.positionRadians;
            zeroed = true;
        });
    }

    public Command forceZeroCommand() {
        return Commands.runOnce(() -> {
            hoodOffset = HoodConstants.kMinimumPosition - inputs.positionRadians;
            zeroed = true;
        }).ignoringDisable(true);
    }

    public Command runFixedPositionCommand(DoubleSupplier angle, DoubleSupplier velocity) {
        return Commands.run(() -> setGoalParameters(angle.getAsDouble(), velocity.getAsDouble()));
    }
}
