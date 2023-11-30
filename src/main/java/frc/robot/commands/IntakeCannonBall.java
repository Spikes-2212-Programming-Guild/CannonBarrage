package frc.robot.commands;

import com.spikes2212.command.genericsubsystem.commands.MoveGenericSubsystem;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.subsystems.IntakeRoller;
import frc.robot.subsystems.Roundabout;
import frc.robot.subsystems.Transfer;

public class IntakeCannonBall extends ParallelCommandGroup {

    private static final double SPEED_SUPPLIER_INTAKE_ROLLER = 0;
    private static final double SPEED_SUPPLIER_ROUNDABOUT = 0;
    private static final double SPEED_SUPPLIER_TRANSFER = 0;
    private static final double INTAKE_ROLLER_TIMEOUT = 0;
    private static final double ROUNDABOUT_TIMEOUT = 0;
    private static final double TRANSFER_TIMEOUT = 0;

    public IntakeCannonBall(IntakeRoller intakeRoller, Roundabout roundabout, Transfer transfer) {
        addCommands(
                new MoveGenericSubsystem(intakeRoller, SPEED_SUPPLIER_INTAKE_ROLLER).withTimeout(INTAKE_ROLLER_TIMEOUT),
                new MoveGenericSubsystem(roundabout, SPEED_SUPPLIER_ROUNDABOUT).withTimeout(ROUNDABOUT_TIMEOUT),
                new MoveGenericSubsystem(transfer, SPEED_SUPPLIER_TRANSFER).withTimeout(TRANSFER_TIMEOUT)
        );
    }
}
