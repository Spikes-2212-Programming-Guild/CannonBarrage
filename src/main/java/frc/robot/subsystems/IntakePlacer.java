package frc.robot.subsystems;

import com.spikes2212.command.DoubleSolenoidSubsystem;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import frc.robot.RobotMap;

public class IntakePlacer extends DoubleSolenoidSubsystem {

    private static final String LEFT_NAMESPACE_NAME = "intake toggle";
    private static final String RIGHT_NAMESPACE_NAME = "intake toggle";

    private static final boolean INVERTED = false;

    private static IntakePlacer leftInstance;
    private static IntakePlacer rightInstance;

    public static IntakePlacer getLeftInstance() {
        if (leftInstance == null) {
            leftInstance = new IntakePlacer(LEFT_NAMESPACE_NAME, new DoubleSolenoid(PneumaticsModuleType.CTREPCM,
                    RobotMap.PWM.LEFT_SOLENOID_INTAKE_PORT_1, RobotMap.PWM.LEFT_SOLENOID_INTAKE_PORT_2));
        }
        return leftInstance;
    }

    public static IntakePlacer getRightInstance() {
        if (rightInstance == null) {
            rightInstance = new IntakePlacer(RIGHT_NAMESPACE_NAME, new DoubleSolenoid(PneumaticsModuleType.CTREPCM,
                    RobotMap.PWM.RIGHT_SOLENOID_INTAKE_PORT_1, RobotMap.PWM.RIGHT_SOLENOID_INTAKE_PORT_2));
        }
        return rightInstance;
    }

    private IntakePlacer(String namespaceName, DoubleSolenoid solenoid) {
        super(namespaceName, solenoid, INVERTED);
    }
}
