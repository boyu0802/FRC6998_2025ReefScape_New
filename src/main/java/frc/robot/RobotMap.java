package frc.robot;

import frc.lib.util.CANDeviceId;

public class RobotMap {
    public static final CANDeviceId CORAL_INTAKE_ID = new CANDeviceId(13);
    public static final CANDeviceId CORAL_WRIST_ID = new CANDeviceId(11);
    public static final CANDeviceId CORAL_WRIST_ENCODER_ID = new CANDeviceId(5); // CanCoder

    public static final int CORAL_INTAKE_LIMITSWITCH_ID = 1;

    public static final CANDeviceId GRAB_INTAKE_ID = new CANDeviceId(4);
    public static final CANDeviceId GRAB_WRIST_ID = new CANDeviceId(3);

    

    public static final CANDeviceId ELEVATOR_ID_LEFT = new CANDeviceId(15);
    public static final CANDeviceId ELEVATOR_ID_RIGHT = new CANDeviceId(9,"rio");
    public static final int ELEVATOR_LIMITSWITCH_ID = 0;

    public static final CANDeviceId HANG_ID = new CANDeviceId(12);
    public static final CANDeviceId CATCH_HANG_ID = new CANDeviceId(2);
    public static final CANDeviceId HANG_ENCODER_ID = new CANDeviceId(6);

    public static final int HANG_LIMITSWITCH_ID = 4;


    public static final int CANDLE_ID = 1;

    public static final String LIMELIGHT_LEFT = "limelight-left";

    public static final String LIMELIGHT_RIGHT = "limelight-right";
    public static final String LIMELIGHT_ELEVATOR = "limelight-elev";



}
