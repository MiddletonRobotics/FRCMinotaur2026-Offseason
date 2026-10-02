package frc.robot.subsystems.shooter.hood;

import org.littletonrobotics.junction.AutoLog;

public interface HoodIO {
    @AutoLog
    public class HoodIOInputs {
        public boolean connected = false;
        public double positionRadians = 0.0;
        public double velocityRadiansPerSecond = 0.0;
        public double appliedVoltage = 0.0;
        public double supplyCurrentAmperes = 0.0;
        public double torqueCurrentAmperes = 0.0;
        public double temperatureCelsius = 0.0;
        public boolean temperatureFault = false;
    }

    public enum HoodIOOutputMode {
        BRAKE,
        COAST,
        VOLTAGE_CONTROL,
        CLOSED_LOOP
    }

    public static class HoodIOOutputs {
        public HoodIOOutputMode mode = HoodIOOutputMode.BRAKE;
        public double appliedVoltage = 0.0;

        public double position = 0.0;
        public double velocity = 0.0;
        
        public double kP = 0.0;
        public double kD = 0.0;
        public double kS = 0.0;
        public double kV = 0.0;
        public double kG = 0.0;
        public double kA = 0.0;
    }

    public void updateInputs(HoodIOInputs inputs);

    public void applyOutputs(HoodIOOutputs outputs);

    public void stop();
}