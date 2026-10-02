package frc.robot.constants;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import frc.minolib.advantagekit.LoggedTunableNumber;
import frc.minolib.hardware.MinoCANDevice;
import frc.robot.Constants;

public class HoodConstants {
    public static final MinoCANDevice kKrakenX44 = new MinoCANDevice(25, Constants.kRioBus);

    public static final LoggedTunableNumber kP = new LoggedTunableNumber("Hood/Gains/kP", 10.0);
    public static final LoggedTunableNumber kD = new LoggedTunableNumber("Hood/Gains/kD", 0.0);
    public static final LoggedTunableNumber kS = new LoggedTunableNumber("Hood/Gains/kS", 0.0);
    public static final LoggedTunableNumber kV = new LoggedTunableNumber("Hood/Gains/kV", 0.0);
    public static final LoggedTunableNumber kG = new LoggedTunableNumber("Hood/Gains/kG", 0.0);
    public static final LoggedTunableNumber kA = new LoggedTunableNumber("Hood/Gains/kA", 0.0);

    public static final boolean kMotorInverted = true;
    public static final double kMotorReduction = (24.0 / 12.0);
    public static final double kMotorStatorCurrentLimit = 120;
    public static final double kMotorSupplyCurrentLimit = 60;
    public static final DCMotor kSimulatedGearbox = DCMotor.getKrakenX44Foc(1);

    public static final double kLength = Units.inchesToMeters(14.0);
    public static final double kMassKilograms = 5.89;
    public static final double kMOI = 0.5 * (0.5) * (Math.pow(Units.inchesToMeters(1.875), 2) + Math.pow(Units.inchesToMeters(2), 2));

    public static final double kHomedPosition = Units.degreesToRadians(12.0);
    public static final double kMinimumPosition = Units.degreesToRadians(12.0);
    public static final double kMaximumPosition = Units.degreesToRadians(30.0);
}
