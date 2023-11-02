package frc.robot.subsystems;

import com.ctre.phoenix.sensors.CANCoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import com.spikes2212.control.FeedForwardSettings;
import com.spikes2212.control.PIDSettings;
import com.spikes2212.dashboard.Namespace;
import com.spikes2212.dashboard.RootNamespace;
import frc.robot.RobotMap;

import java.util.function.Supplier;
// https://cdn.discordapp.com/attachments/927272978356510721/1169660514633273424/0jnqr6h825vb1.png?ex=6556363d&is=6543c13d&hm=15df0944274ba2c224f0aa2a6e8ffffb181e8710de04937458dd89cec58940aa&
public class SwerveModuleHolder {

    private static final RootNamespace namespace = new RootNamespace("module holder");

    private final Namespace drivePIDNamespace = namespace.addChild("drive pid");
    private final Supplier<Double> kPDrive = drivePIDNamespace.addConstantDouble("drive kP", 0);
    private final Supplier<Double> kIDrive = drivePIDNamespace.addConstantDouble("drive kI", 0);
    private final Supplier<Double> kDDrive = drivePIDNamespace.addConstantDouble("drive kD", 0);
    private final Supplier<Double> driveTolerance = drivePIDNamespace.addConstantDouble("drive tolerance", 0);
    private final Supplier<Double> driveWaitTime = drivePIDNamespace.addConstantDouble("drive wait time", 0);
    private final PIDSettings drivePIDSettings =
            new PIDSettings(kPDrive, kIDrive, kDDrive, driveTolerance, driveWaitTime);

    private final Supplier<Double> kSDrive = drivePIDNamespace.addConstantDouble("drive kS", 0);
    private final Supplier<Double> kVDrive = drivePIDNamespace.addConstantDouble("drive kV", 0);
    private final Supplier<Double> kADrive = drivePIDNamespace.addConstantDouble("drive kA", 0);
    private final FeedForwardSettings driveFeedForwardSettings = new FeedForwardSettings(kSDrive, kVDrive, kADrive);

    private final Namespace turnPIDNamespace = namespace.addChild("turn pid");
    private final Supplier<Double> kPTurn = turnPIDNamespace.addConstantDouble("turn kP", 0);
    private final Supplier<Double> kITurn = turnPIDNamespace.addConstantDouble("turn kI", 0);
    private final Supplier<Double> kDTurn = turnPIDNamespace.addConstantDouble("turn kD", 0);
    private final Supplier<Double> turnTolerance = turnPIDNamespace.addConstantDouble("turn tolerance", 0);
    private final Supplier<Double> turnWaitTime = turnPIDNamespace.addConstantDouble("turn wait time", 0);
    private final PIDSettings turnPIDSettings =
            new PIDSettings(kPTurn, kITurn, kDTurn, turnTolerance, turnWaitTime);

    private static SwerveModuleImpl frontLeft;
    private static SwerveModuleImpl frontRight;
    private static SwerveModuleImpl backLeft;
    private static SwerveModuleImpl backRight;

    public SwerveModuleImpl getFrontLeft() {
        if (frontLeft == null) {
            frontLeft = new SwerveModuleImpl("front left",
                    new CANSparkMax(RobotMap.CAN.FRONT_LEFT_DRIVE_SPARKMAX, CANSparkMaxLowLevel.MotorType.kBrushless),
                    new CANSparkMax(RobotMap.CAN.FRONT_LEFT_TURN_SPARKMAX, CANSparkMaxLowLevel.MotorType.kBrushless),
                    new CANCoder(RobotMap.CAN.FRONT_LEFT_ABSOLUTE_ENCODER), 0, driveFeedForwardSettings,
                    drivePIDSettings, turnPIDSettings);
        }
        return frontLeft;
    }

    public SwerveModuleImpl getFrontRight() {
        if (frontRight == null) {
            frontRight = new SwerveModuleImpl("front right",
                    new CANSparkMax(RobotMap.CAN.FRONT_RIGHT_DRIVE_SPARKMAX, CANSparkMaxLowLevel.MotorType.kBrushless),
                    new CANSparkMax(RobotMap.CAN.FRONT_RIGHT_TURN_SPARKMAX, CANSparkMaxLowLevel.MotorType.kBrushless),
                    new CANCoder(RobotMap.CAN.FRONT_RIGHT_ABSOLUTE_ENCODER), 0, driveFeedForwardSettings,
                    drivePIDSettings, turnPIDSettings);
        }
        return frontRight;
    }

    public SwerveModuleImpl getBackLeft() {
        if (backLeft == null) {
            backLeft = new SwerveModuleImpl("back left",
                    new CANSparkMax(RobotMap.CAN.BACK_LEFT_DRIVE_SPARKMAX, CANSparkMaxLowLevel.MotorType.kBrushless),
                    new CANSparkMax(RobotMap.CAN.BACK_LEFT_TURN_SPARKMAX, CANSparkMaxLowLevel.MotorType.kBrushless),
                    new CANCoder(RobotMap.CAN.BACK_LEFT_ABSOLUTE_ENCODER), 0, driveFeedForwardSettings,
                    drivePIDSettings, turnPIDSettings);
        }
        return backLeft;
    }

    public SwerveModuleImpl getBackRight() {
        if (backRight == null) {
            backRight = new SwerveModuleImpl("back right",
                    new CANSparkMax(RobotMap.CAN.BACK_RIGHT_DRIVE_SPARKMAX, CANSparkMaxLowLevel.MotorType.kBrushless),
                    new CANSparkMax(RobotMap.CAN.BACK_RIGHT_TURN_SPARKMAX, CANSparkMaxLowLevel.MotorType.kBrushless),
                    new CANCoder(RobotMap.CAN.BACK_RIGHT_ABSOLUTE_ENCODER), 0, driveFeedForwardSettings,
                    drivePIDSettings, turnPIDSettings);
        }
        return backRight;
    }
}
