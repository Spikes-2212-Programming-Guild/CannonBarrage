package frc.robot.subsystems;

import com.ctre.phoenix.sensors.AbsoluteSensorRange;
import com.ctre.phoenix.sensors.CANCoder;
import com.revrobotics.CANSparkMax;
import com.spikes2212.command.DashboardedSubsystem;
import com.spikes2212.control.FeedForwardController;
import com.spikes2212.control.FeedForwardSettings;
import com.spikes2212.control.PIDSettings;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;

public class SwerveModuleImpl extends DashboardedSubsystem implements SwerveModule {

    private static final int PID_SLOT = 0;
    private static final double STEERING_GEAR_RATIO = 12.8;
    private final CANSparkMax driveController;
    private final CANSparkMax turnController;
    private final CANCoder absoluteEncoder;
    private final double offset;
    private final FeedForwardController driveFeedForwardController;
    private final FeedForwardSettings driveFeedForwardSettings;
    private final PIDSettings drivePIDSettings;
    private final PIDSettings turnPIDSettings;

    public SwerveModuleImpl(String namespaceName, CANSparkMax driveController, CANSparkMax turnController,
                            CANCoder absoluteEncoder, double offset, FeedForwardSettings driveFeedForwardSettings,
                            PIDSettings drivePIDSettings, PIDSettings turnPIDSettings) {
        super(namespaceName);
        this.driveController = driveController;
        this.turnController = turnController;
        this.absoluteEncoder = absoluteEncoder;
        this.offset = offset;
        this.driveFeedForwardSettings = driveFeedForwardSettings;
        this.drivePIDSettings = drivePIDSettings;
        this.turnPIDSettings = turnPIDSettings;
        this.driveFeedForwardController = new FeedForwardController(driveFeedForwardSettings, FeedForwardController.DEFAULT_PERIOD);
        configureDriveController();
        configureTurnController();
        configureAbsoluteEncoder();
        configureRelativeEncoder();
    }

    private void configureDriveController() {
        driveController.getPIDController().setP(drivePIDSettings.getkP());
        driveController.getPIDController().setI(drivePIDSettings.getkI());
        driveController.getPIDController().setD(drivePIDSettings.getkD());
    }

    private void configureTurnController() {
        turnController.getPIDController().setP(turnPIDSettings.getkP());
        turnController.getPIDController().setI(turnPIDSettings.getkI());
        turnController.getPIDController().setD(turnPIDSettings.getkD());
    }

    private void configureAbsoluteEncoder() {
        absoluteEncoder.configFactoryDefault();
        absoluteEncoder.configMagnetOffset(offset);
        absoluteEncoder.configAbsoluteSensorRange(AbsoluteSensorRange.Signed_PlusMinus180);
    }

    private void configureRelativeEncoder() {
        turnController.getEncoder().setPositionConversionFactor((1 / STEERING_GEAR_RATIO) * 180);
        turnController.getEncoder().setPosition(getAngle());
    }

    @Override
    public void set(SwerveModuleState state, boolean usePID) {
        state = SwerveModuleState.optimize(state, Rotation2d.fromDegrees(absoluteEncoder.getAbsolutePosition()));
        setAngle(state.angle.getDegrees());
        setSpeed(state.speedMetersPerSecond, usePID);
    }

    //angle between -180 and 180
    private void setAngle(double angle) {
        configureTurnController();
        turnController.getPIDController().setReference(angle, CANSparkMax.ControlType.kPosition, PID_SLOT);
    }

    //speed - m/s
    private void setSpeed(double speed, boolean usePID) {
        if (usePID) {
            configureDriveController();
            double feedForward = driveFeedForwardController.calculate(speed);
            driveController.getPIDController().setReference(speed, CANSparkMax.ControlType.kVelocity, PID_SLOT,
                    feedForward);
        } else {
            driveController.set(speed / DrivetrainImpl.MAX_SPEED);
        }
    }

    @Override
    public double getAngle() {
        return absoluteEncoder.getAbsolutePosition();
    }

    @Override
    public void configureDashboard() {
    }
}
