package frc.robot;

public class RobotMap {

    public interface CAN {

        int FRONT_LEFT_DRIVE_SPARKMAX = 2;
        int FRONT_LEFT_TURN_SPARKMAX = 1;
        int FRONT_LEFT_ABSOLUTE_ENCODER = 3;

        int FRONT_RIGHT_DRIVE_SPARKMAX = -1;
        int FRONT_RIGHT_TURN_SPARKMAX = -1;
        int FRONT_RIGHT_ABSOLUTE_ENCODER = -1;

        int BACK_LEFT_DRIVE_SPARKMAX = -1;
        int BACK_LEFT_TURN_SPARKMAX = -1;
        int BACK_LEFT_ABSOLUTE_ENCODER = -1;

        int BACK_RIGHT_DRIVE_SPARKMAX = -1;
        int BACK_RIGHT_TURN_SPARKMAX = -1;
        int BACK_RIGHT_ABSOLUTE_ENCODER = -1;
    }

    public interface DIO {

    }

    public interface PWM {

    }

    public interface AIN {

    }

    public interface PCM {

    }
}
