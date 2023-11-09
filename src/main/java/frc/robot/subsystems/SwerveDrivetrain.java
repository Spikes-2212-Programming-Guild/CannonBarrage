package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Subsystem;

import java.util.function.Supplier;

public interface SwerveDrivetrain extends Subsystem {

    void drive(Supplier<Double> xSpeed, Supplier<Double> ySpeed, Supplier<Double> rotationSpeed,
               boolean fieldRelative, boolean usePID);

    void stop();

    void resetGyro();
}
