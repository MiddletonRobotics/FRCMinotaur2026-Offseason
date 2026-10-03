package frc.robot.subsystems.shooter.flywheel;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;

import frc.robot.Constants;
import frc.robot.constants.FlywheelConstants;

public class FlywheelIOSimulation implements FlywheelIO {
    private final DCMotor gearbox;
    private final DCMotorSim simulation;
    private double appliedVoltage = 0.0;

    private PIDController controller = new PIDController(0.6, 0, 0, Constants.kLoopPeriodSeconds);

    private double currentOutput = 0.0;
    private double currentOutputAsVoltage = 0.0;
    private double lastVelocity = 0.0;

    public FlywheelIOSimulation() {
        gearbox = FlywheelConstants.kSimulatedGearbox;
        simulation = new DCMotorSim(LinearSystemId.createDCMotorSystem(gearbox, .3, 1), gearbox);
    }

    @Override
    public void updateInputs(FlywheelIOInputs inputs) {
        currentOutputAsVoltage = MathUtil.clamp(gearbox.getVoltage(currentOutput, simulation.getAngularVelocityRadPerSec()), -12.0, 12.0);
        appliedVoltage = currentOutputAsVoltage;

        simulation.setInputVoltage(MathUtil.clamp(appliedVoltage, -12.0, 12.0));
        simulation.update(Constants.kLoopBackTimeSeconds);

        lastVelocity = inputs.velocity;

        inputs.primaryConnected = true;
        inputs.followerOneConnected = true;
        inputs.followerTwoConnected = true;
        inputs.followerThreeConnected = true;

        inputs.position = simulation.getAngularPositionRad();
        inputs.velocity = simulation.getAngularVelocityRadPerSec();
        inputs.appliedVoltage = appliedVoltage;
        inputs.torqueCurrent = currentOutput;

        inputs.primarySupplyCurrent = simulation.getCurrentDrawAmps();
        inputs.followerOneSupplyCurrent = simulation.getCurrentDrawAmps();
        inputs.followerTwoSupplyCurrent = simulation.getCurrentDrawAmps();
        inputs.followerThreeSupplyCurrent = simulation.getCurrentDrawAmps();

        inputs.primaryTemperature = 0.0;
        inputs.followerOneTemperature = 0.0;
        inputs.followerTwoTemperature = 0.0;
        inputs.followerThreeTemperature = 0.0;
    }

    @Override
    public void applyOutputs(FlywheelIOOutputs outputs) {
        if (outputs.mode == FlywheelIOOutputMode.COAST) {
            currentOutput = 0.0;
        } else {
            controller.setSetpoint(outputs.goalVelocity);
            currentOutput = controller.calculate(lastVelocity);
        }
    }

    @Override
    public void stop() {
        appliedVoltage = 0.0;
        currentOutput = 0.0;
    }
}
