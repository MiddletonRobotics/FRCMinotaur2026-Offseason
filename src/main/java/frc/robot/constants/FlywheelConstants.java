package frc.robot.constants;

import edu.wpi.first.math.system.plant.DCMotor;
import frc.minolib.advantagekit.LoggedTunableNumber;
import frc.minolib.hardware.MinoCANDevice;
import frc.robot.Constants;

public class FlywheelConstants {
    public static final MinoCANDevice kPrimaryMotor = new MinoCANDevice(20, Constants.kCANivoreBus);
    public static final MinoCANDevice kFollowerOneMotor = new MinoCANDevice(21, Constants.kCANivoreBus);
    public static final MinoCANDevice kFollowerTwoMotor = new MinoCANDevice(22, Constants.kCANivoreBus);
    public static final MinoCANDevice kFollowerThreeMotor = new MinoCANDevice(23, Constants.kCANivoreBus);

    public static final LoggedTunableNumber kP = new LoggedTunableNumber("Flywheel/Gains/kP", 10.0);
    public static final LoggedTunableNumber kD = new LoggedTunableNumber("Flywheel/Gains/kD", 0.0);
    public static final LoggedTunableNumber kS = new LoggedTunableNumber("Flywheel/Gains/kS", 0.0);
    public static final LoggedTunableNumber kV = new LoggedTunableNumber("Flywheel/Gains/kV", 0.0);
    public static final LoggedTunableNumber kG = new LoggedTunableNumber("Flywheel/Gains/kG", 0.0);
    public static final LoggedTunableNumber kA = new LoggedTunableNumber("Flywheel/Gains/kA", 0.0);

    public static final boolean kPrimaryMotorInverted = true;
    public static final boolean kOppositeMotorInverted = false;
    public static final double kMotorReduction = (24.0 / 12.0);
    public static final double kMotorStatorCurrentLimit = 120;
    public static final double kMotorSupplyCurrentLimit = 60;
    public static final DCMotor kSimulatedGearbox = DCMotor.getKrakenX60Foc(4);

    public static final LoggedTunableNumber kBangBangConstant = new LoggedTunableNumber("Flywheel/BangBangConstant", 2.0);
    public static final LoggedTunableNumber kBangBangMinDistance = new LoggedTunableNumber("Flywheel/BangBangMinDistance", 3.25);
    public static final double kAccelerationFilterTimeConstantSeconds = 0.050;
}
