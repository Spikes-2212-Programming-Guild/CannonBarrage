package frc.robot.subsystems;

import java.util.function.Supplier;

public interface Drivetrain {

    void drive(Supplier<Double> xSpeed, Supplier<Double> ySpeed, Supplier<Double> rotationSpeed,
               boolean fieldRelative, boolean usePID);

    void resetGyro();
}
