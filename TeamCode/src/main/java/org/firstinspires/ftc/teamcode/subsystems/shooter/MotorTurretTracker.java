package org.firstinspires.ftc.teamcode.subsystems.shooter;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

@Config
public class MotorTurretTracker extends SubsystemBase {
    private final DcMotorEx turret;
    private boolean enabled = false;
    private double targetX = 0.0;
    private double targetY = 0.0;

    // ========== LIMITS ==========
    public static double MOTOR_TICKS_MIN = 0.0;
    public static double MOTOR_TICKS_MAX = 0.0;
    public static double ENCODER_ABSOLUTE_OFFSET = 0.0;

    public static double TURRET_WINDOW_MIN_DEG = 0.0;
    public static double TURRET_WINDOW_MAX_DEG = 360.0;

    // ========== ODO TOGGLES ==========
    public static boolean SWAP_XY = false;
    public static boolean INVERT_X = false;
    public static boolean INVERT_Y = false;

    public static double HEADING_OFFSET_DEG = 0.0;
    // Robot-frame offset from odom origin to turret pivot (inches)
    // +X forward, +Y left
    public static double TURRET_OFFSET_X_IN = 0.0;
    public static double TURRET_OFFSET_Y_IN = 0.0;

    // ========== Calibration (deg -> servo pos) ==========
    public static double A0_DEG = 0.0;   public static double P0 = 0.31;
    public static double A1_DEG = 45.0;  public static double P1 = 0.44;
    public static double A2_DEG = 90.0;  public static double P2 = 0.567;
    public static double A3_DEG = 135.0; public static double P3 = 0.7;
    public static double A4_DEG = 180.0; public static double P4 = 0.83;

    // ========== TELEMETRY ==========
    private double lastTargetFieldDeg = 0.0;
    private double lastRobotFieldHeadingDeg = 0.0;
    private double lastTurretRobotDegCmd = 90.0;
    private double lastServoPosCmd = 0.505;

    public MotorTurretTracker(HardwareMap hardwareMap) {
        turret = hardwareMap.get(DcMotorEx.class, "turret");

    }
}
