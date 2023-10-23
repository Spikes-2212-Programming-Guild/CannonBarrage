package frc.robot.subsystems;

import com.spikes2212.command.DashboardedSubsystem;
import com.spikes2212.dashboard.Namespace;

import java.util.function.Supplier;

public abstract class SwerveDrivetrain extends DashboardedSubsystem {

    public SwerveDrivetrain(Namespace namespace) {
        super(namespace);
    }

    public abstract void drive(Supplier<Double> xSpeed, Supplier<Double> ySpeed, Supplier<Double> rotationSpeed);

    public abstract void stop();

    public abstract void resetGyro();
}
