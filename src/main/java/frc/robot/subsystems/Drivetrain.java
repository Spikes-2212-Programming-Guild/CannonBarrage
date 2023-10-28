package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Subsystem;

import java.util.function.Supplier;

public interface Drivetrain extends Subsystem {

    void drive(Supplier<Double> xSpeed, Supplier<Double> ySpeed, Supplier<Double> rotationSpeed,
               boolean fieldRelative, boolean usePID);

    void resetGyro();
}
