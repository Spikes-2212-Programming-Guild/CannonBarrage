// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.spikes2212.dashboard.RootNamespace;
import com.spikes2212.util.PlaystationControllerWrapper;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import frc.robot.subsystems.SwerveModuleHolder;
import frc.robot.subsystems.SwerveModuleImpl;

import java.util.function.Supplier;

public class Robot extends TimedRobot {

    private static final RootNamespace namespace = new RootNamespace("5");
    private SwerveModuleImpl module;
    private PlaystationControllerWrapper ps = new PlaystationControllerWrapper(0);
    private final Supplier<Double> angle = namespace.addConstantDouble("angle", Math.atan(ps.getLeftX() / ps.getLeftY()));
    private final Supplier<Double> speed = namespace.addConstantDouble("speed", Math.sqrt(
            ps.getLeftX() * ps.getLeftX() * 9 + ps.getLeftY() * ps.getLeftY() * 9));

    @Override
    public void robotInit() {
        module = new SwerveModuleHolder().getFrontLeft();
    }

    @Override
    public void robotPeriodic() {
        module.periodic();
        CommandScheduler.getInstance().run();
    }

    @Override
    public void disabledInit() {

    }

    @Override
    public void disabledPeriodic() {

    }

    @Override
    public void autonomousInit() {

    }

    @Override
    public void autonomousPeriodic() {

    }

    @Override
    public void teleopInit() {
        OI oi = new OI();
        RunCommand command = new RunCommand(() -> module.set(new SwerveModuleState(speed.get(),
                new Rotation2d(angle.get())), false), module);
//        module.setDefaultCommand(command);
    }

    @Override
    public void teleopPeriodic() {

    }

    @Override
    public void testInit() {
        CommandScheduler.getInstance().cancelAll();
    }

    @Override
    public void testPeriodic() {

    }

    @Override
    public void simulationInit() {

    }

    @Override
    public void simulationPeriodic() {

    }
}
