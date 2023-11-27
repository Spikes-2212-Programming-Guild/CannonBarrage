package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.subsystems.IntakePlacer;

public class ToggleIntake extends ParallelCommandGroup {

    public ToggleIntake(IntakePlacer leftIntakePlacer, IntakePlacer rightIntakePlacer) {
        addCommands(leftIntakePlacer.openSolenoid(), rightIntakePlacer.openSolenoid());
    }
}
