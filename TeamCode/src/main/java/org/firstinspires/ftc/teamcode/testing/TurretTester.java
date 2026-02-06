//package org.firstinspires.ftc.teamcode.testing;
//
//import com.acmerobotics.dashboard.FtcDashboard;
//import com.acmerobotics.dashboard.config.Config;
//import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
//import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
//import com.qualcomm.robotcore.eventloop.opmode.OpMode;
//import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
//
//import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
//import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
//import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
//import org.firstinspires.ftc.teamcode.subsystems.shooter.Turret;
//import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
//
//@Config
//@TeleOp(name = "Test: Turret Aim at Field Point (Pedro coords)", group = "Test")
//public class TurretTester extends OpMode {
//
//    // RC config names
//    public static String TURRET_SERVO_NAME = "turret";
//    public static String PINPOINT_NAME = "pinpoint";
//
//    // Enable + direction
//    public static boolean ENABLE = true;
//    public static boolean INVERT_PINPOINT_HEADING = false;
//    public static boolean INVERT_TURRET = true;
//
//    // Coordinate mapping (Pinpoint -> Pedro)
//    public static boolean SWAP_XY = false;
//    public static boolean INVERT_X = false;
//    public static boolean INVERT_Y = false;
//
//    // Target point (Pedro field coords, inches)
//    public static double TARGET_X_IN = 144.0;
//    public static double TARGET_Y_IN = 144.0;
//
//    // Turret allowed range (deg). Use 0..360 if you like that representation.
//    public static double MIN_TURRET_DEG = 0.0;
//    public static double MAX_TURRET_DEG = 360.0;
//
//    // Servo endpoints for those limits
//    public static double SERVO_MIN = 0.02;
//    public static double SERVO_MAX = 0.98;
//
//    // Offset so turret "0" matches robot forward
//    public static double TURRET_ZERO_OFFSET_DEG = 0.0;
//
//    // Deadband
//    public static double HEADING_DEADBAND_DEG = 1.0;
//    public static double HEADING_HYSTERESIS_DEG = 0.3;
//
//    // Rate limit
//    public static double MAX_SERVO_SPEED_PER_SEC = 1.0;
//
//    // Set starting pose (0,0,0) once from dashboard
//    public static boolean SET_START_POSE_0_0 = false;
//
//    // If you change tunables and want to re-seed deadband/reference
//    public static boolean RESET_STATE = false;
//
//    private Turret turret;
//    private GoBildaPinpointDriver pinpoint;
//    private MultipleTelemetry tele;
//    private FtcDashboard dash;
//
//    @Override
//    public void init() {
//        dash = FtcDashboard.getInstance();
//        tele = new MultipleTelemetry(telemetry, dash.getTelemetry());
//        dash.setTelemetryTransmissionInterval(25);
//
////        pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, PINPOINT_NAME);
//        turret = new Turret(hardwareMap, TURRET_SERVO_NAME);
//
//        tele.addLine("Pedro coords: (0,0) bottom-left, (144,144) top-right. Target default = (144,144).");
//        tele.addLine("If aiming is wrong: toggle SWAP_XY / INVERT_X / INVERT_Y first.");
//        tele.update();
//    }
//
//    @Override
//    public void loop() {
//        if (SET_START_POSE_0_0) {
//            // Start pose should be (0,0) with heading 0 deg (+X).
//            pinpoint.setPosition(new Pose2D(DistanceUnit.MM, 0.0, 0.0, AngleUnit.RADIANS, 0.0));
//            turret.resetControllerState();
//            SET_START_POSE_0_0 = false;
//        }
//
//        if (RESET_STATE) {
//            turret.resetControllerState();
//            RESET_STATE = false;
//        }
//
//        turret.setEnabled(ENABLE);
//
//        turret.setTurretLimitsDeg(MIN_TURRET_DEG, MAX_TURRET_DEG);
//        turret.setServoLimits(SERVO_MIN, SERVO_MAX);
//        turret.setTurretZeroOffsetDeg(TURRET_ZERO_OFFSET_DEG);
//
//        turret.setInvertPinpointHeading(INVERT_PINPOINT_HEADING);
//        turret.setInvertTurret(INVERT_TURRET);
//
//        turret.setCoordinateMapping(SWAP_XY, INVERT_X, INVERT_Y);
//
//        turret.setHeadingDeadbandDeg(HEADING_DEADBAND_DEG);
//        turret.setHeadingHysteresisDeg(HEADING_HYSTERESIS_DEG);
//
//        turret.setMaxServoSpeedPerSec(MAX_SERVO_SPEED_PER_SEC);
//
//        turret.setTrackFieldPointInches(TARGET_X_IN, TARGET_Y_IN);
//
//        // Manual call since this is a plain OpMode
//        turret.periodic();
//
//        // Telemetry: show current pinpoint pose (as used) + target
//        tele.addLine("---- Pinpoint (Pedro-aligned) ----");
//        tele.addData("robotX", "%.2f", turret.getRobotXIn());
//        tele.addData("robotY", "%.2f", turret.getRobotYIn());
//        tele.addData("robotHeadingDeg", "%.2f", turret.getRobotHeadingDeg());
//
//        tele.addLine("---- Target ----");
//        tele.addData("targetX", "%.2f", turret.getTargetXIn());
//        tele.addData("targetY", "%.2f", turret.getTargetYIn());
//        tele.addData("angleToTargetDeg(field)", "%.2f", turret.getAngleToTargetDeg());
//
//        tele.addLine("---- Turret Command ----");
//        tele.addData("desiredTurretDeg", "%.2f", turret.getDesiredTurretDeg());
//        tele.addData("wrappedTurretDeg", "%.2f", turret.getWrappedTurretDeg());
//        tele.addData("servoPos", "%.4f", turret.getServoPos());
//        tele.update();
//
//        TelemetryPacket packet = new TelemetryPacket();
//        packet.put("robotX", turret.getRobotXIn());
//        packet.put("robotY", turret.getRobotYIn());
//        packet.put("robotHeadingDeg", turret.getRobotHeadingDeg());
//        packet.put("targetX", turret.getTargetXIn());
//        packet.put("targetY", turret.getTargetYIn());
//        packet.put("angleToTargetDeg", turret.getAngleToTargetDeg());
//        packet.put("wrappedTurretDeg", turret.getWrappedTurretDeg());
//        packet.put("servoPos", turret.getServoPos());
//        dash.sendTelemetryPacket(packet);
//    }
//}
