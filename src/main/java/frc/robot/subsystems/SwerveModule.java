package frc.robot.subsystems;

import edu.wpi.first.math.kinematics.SwerveModuleState;

public interface SwerveModule {

    void set(SwerveModuleState state, boolean usePID);

    double getAngle();
}
