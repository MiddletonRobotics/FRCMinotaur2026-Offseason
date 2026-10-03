package frc.robot.subsystems.shooter.flywheel;

import org.littletonrobotics.junction.AutoLog;

public interface FlywheelIO {
    @AutoLog
    public class FlywheelIOInputs {
        public boolean primaryConnected = false;
        public boolean followerOneConnected = false;
        public boolean followerTwoConnected = false;
        public boolean followerThreeConnected = false;

        public double position = 0.0;
        public double velocity = 0.0;
        public double acceleration = 0.0;
        public double appliedVoltage = 0.0;
        public double torqueCurrent = 0.0;

        public double primarySupplyCurrent = 0.0;
        public double followerOneSupplyCurrent = 0.0;
        public double followerTwoSupplyCurrent = 0.0;
        public double followerThreeSupplyCurrent = 0.0;

        public double primaryTemperature = 0.0;
        public double followerOneTemperature = 0.0;
        public double followerTwoTemperature = 0.0;
        public double followerThreeTemperature = 0.0;
    }

    public enum FlywheelIOOutputMode {
        BRAKE,
        COAST,
        VOLTAGE_CONTROL,
        VELOCITY_CONTROL
    }

    public static class FlywheelIOOutputs {
        public FlywheelIOOutputMode mode = FlywheelIOOutputMode.BRAKE;
        public double appliedVoltage = 0.0;
        public double goalVelocity = 0.0;

        public double kP = 0.0;
        public double kD = 0.0;
        public double kS = 0.0;
        public double kV = 0.0;
        public double kA = 0.0;
    }

    public void updateInputs(FlywheelIOInputs inputs);

    public void applyOutputs(FlywheelIOOutputs outputs);

    public void stop();
}
