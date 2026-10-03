package frc.robot.subsystems.shooter.flywheel;

import static edu.wpi.first.units.Units.Volts;

import java.util.function.DoubleSupplier;

import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;

import frc.minolib.advantagekit.LoggedTracer;
import frc.minolib.advantagekit.LoggedTunableNumber;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.constants.FlywheelConstants;
import frc.robot.subsystems.shooter.flywheel.FlywheelIO.FlywheelIOOutputMode;
import frc.robot.subsystems.shooter.flywheel.FlywheelIO.FlywheelIOOutputs;

import lombok.Getter;
import lombok.experimental.Accessors;

public class Flywheel extends SubsystemBase {
    private final FlywheelIO io;
    private final FlywheelIOInputsAutoLogged inputs = new FlywheelIOInputsAutoLogged();
    private final FlywheelIOOutputs outputs = new FlywheelIOOutputs();

    private final Debouncer primaryConnectedDebouncer = new Debouncer(0.5, Debouncer.DebounceType.kFalling);
    private final Debouncer followerOneConnectedDebouncer = new Debouncer(0.5, Debouncer.DebounceType.kFalling);
    private final Debouncer followerTwoConnectedDebouncer = new Debouncer(0.5, Debouncer.DebounceType.kFalling);
    private final Debouncer followerThreeConnectedDebouncer = new Debouncer(0.5, Debouncer.DebounceType.kFalling);

    private final Alert primaryDisconnectedAlert;
    private final Alert followerOneDisconnectedAlert;
    private final Alert followerTwoDisconnectedAlert;
    private final Alert followerThreeDisconnectedAlert;

    private static final LoggedTunableNumber maxAcceleration = new LoggedTunableNumber("Flywheel/MaxAccelerationRadiansPerSecond2", 350.0);

    @Getter private boolean withinTolerancekS = true;
    @Getter private boolean withinTolerancekV = true;

    private double goalVelocity = 0.0;
    private double setpointVelocity = 0.0;
    private double filteredAcceleration = 0.0; // Used for kA
    private boolean accelerationFilterInitialized = false;

    private final SysIdRoutine sysIdRoutine;

    @Getter
    @Accessors(fluent = true)
    @AutoLogOutput(key = "Flywheel/AtGoal")
    private boolean atGoal = false;

    public Flywheel(FlywheelIO io) {
        this.io = io;

        primaryDisconnectedAlert = new Alert("Flywheel Primary Motor Disconnected", AlertType.kWarning);
        followerOneDisconnectedAlert = new Alert("Flywheel Follower 1 Motor Disconnected", AlertType.kWarning);
        followerTwoDisconnectedAlert = new Alert("Flywheel Follower 2 Motor Disconnected", AlertType.kWarning);
        followerThreeDisconnectedAlert = new Alert("Flywheel Follower 3 Motor Disconnected", AlertType.kWarning);

        sysIdRoutine = new SysIdRoutine(
            new SysIdRoutine.Config(
                null,
                null,
                null,
                state -> Logger.recordOutput("Flywheel/SysIdState", state.toString())),
            new SysIdRoutine.Mechanism(
                voltage -> runVoltage(voltage.in(Volts)),
                null,
                this
            )
        );
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("Flywheel", inputs);

        outputs.kP = FlywheelConstants.kP.get();
        outputs.kD = FlywheelConstants.kD.get();

        if (DriverStation.isDisabled()) {
            stop();
        }

        primaryDisconnectedAlert.set(Robot.showHardwareAlerts() && !primaryConnectedDebouncer.calculate(inputs.primaryConnected));
        followerOneDisconnectedAlert.set(Robot.showHardwareAlerts() && !followerOneConnectedDebouncer.calculate(inputs.followerOneConnected));
        followerTwoDisconnectedAlert.set(Robot.showHardwareAlerts() && !followerTwoConnectedDebouncer.calculate(inputs.followerTwoConnected));
        followerThreeDisconnectedAlert.set(Robot.showHardwareAlerts() && !followerThreeConnectedDebouncer.calculate(inputs.followerThreeConnected));

        Robot.batteryLogger.reportCurrentUsage(
            "Flywheel", 
            inputs.primaryConnected ? inputs.primarySupplyCurrent : 0.0, 
            inputs.followerOneConnected ? inputs.followerOneSupplyCurrent : 0.0,
            inputs.followerTwoConnected ? inputs.followerTwoSupplyCurrent : 0.0,
            inputs.followerThreeConnected ? inputs.followerThreeSupplyCurrent : 0.0
        );

        SmartDashboard.putString("Flywheel Speed", String.format("%.0f", inputs.velocity));
        SmartDashboard.putBoolean("Flywheel At Goal", atGoal);

        io.applyOutputs(outputs);

        LoggedTracer.record("FlywheelPeriodic");
    }

    private void runVelocity(double velocityRadiansPerSecond, boolean bangBang) {
        double maxVelocityStep =  maxAcceleration.get() * Constants.kLoopPeriodSeconds;
        double velocityError = velocityRadiansPerSecond - setpointVelocity;
        double rawAcceleration;

        if (Math.abs(velocityError) <= maxVelocityStep) {
            setpointVelocity = velocityRadiansPerSecond;
            rawAcceleration = velocityError / Constants.kLoopPeriodSeconds;
        } else {
            setpointVelocity += Math.copySign(maxVelocityStep, velocityError);
            rawAcceleration = Math.copySign(maxAcceleration.get(), velocityError);
        }

        if (!accelerationFilterInitialized) {
            filteredAcceleration = rawAcceleration;
            accelerationFilterInitialized = true;
        } else {
            filteredAcceleration += (rawAcceleration - filteredAcceleration) * Constants.kLoopPeriodSeconds / FlywheelConstants.kAccelerationFilterTimeConstantSeconds;
        }

        double feedforward = Math.signum(setpointVelocity) * FlywheelConstants.kS.get() + setpointVelocity * FlywheelConstants.kV.get() + filteredAcceleration * FlywheelConstants.kA.get();
        goalVelocity = velocityRadiansPerSecond;

        if (bangBang) {
            outputs.mode = FlywheelIOOutputMode.VOLTAGE_CONTROL;
            outputs.appliedVoltage = feedforward;

            if (inputs.velocity < setpointVelocity) {
                outputs.appliedVoltage *= FlywheelConstants.kBangBangConstant.get();
            }
        } else {
            outputs.mode = FlywheelIOOutputMode.VELOCITY_CONTROL;
            outputs.goalVelocity = setpointVelocity;
            outputs.appliedVoltage = feedforward;
        }

        Logger.recordOutput("Flywheel/GoalVelocity", goalVelocity);
        Logger.recordOutput("Flywheel/SetpointVelocity", setpointVelocity);
        Logger.recordOutput("Flywheel/SetpointAcceleration", filteredAcceleration);
        Logger.recordOutput("Flywheel/Feedforward", feedforward);
        Logger.recordOutput("Flywheel/BangBang", bangBang);
    }

    private void runVoltage(double voltage) {
        outputs.mode = FlywheelIOOutputMode.VOLTAGE_CONTROL;
        outputs.appliedVoltage = voltage;
    }

    private void stop() {
        outputs.mode = FlywheelIOOutputMode.COAST;
        outputs.goalVelocity = 0.0;
        atGoal = false;
        setpointVelocity = getVelocity();
    }

    public double getVelocity() {
        return inputs.velocity;
    }

    public boolean withinTolerance(double toleranceRadPerSecond) {
        return Math.abs(inputs.velocity - goalVelocity) < toleranceRadPerSecond;
    }

    public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
        return sysIdRoutine.quasistatic(direction);
    }

    public Command sysIdDynamic(SysIdRoutine.Direction direction) {
        return sysIdRoutine.dynamic(direction);
    }

    public Command runFixedCommand(DoubleSupplier velocity, boolean bangbang) {
        return Commands.runEnd(() -> runVelocity(velocity.getAsDouble(), false), this::stop);
    }

    public Command runIdleCommand(DoubleSupplier velocity) {
        return Commands.runEnd(() -> runVelocity(velocity.getAsDouble(), false), this::stop);
    }

    public Command stopCommand() {
        return Commands.run(this::stop);
    }
}
