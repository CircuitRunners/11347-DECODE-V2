package org.firstinspires.ftc.teamcode.subsystems.shooter;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

@Config
public class MotorTurretTracker extends SubsystemBase {
    // NOTE: ANGLE IN REFERENCE TO INTAKE WHERE INTAKE IS FACING THE WALL OF
    // THE GOAL WE ARE SHOOTING ON (RED IS RIGHT WALL, BLUE LEFT), NOT STARTING POSE.
    // MIN and MAX deg are the maximum and minimum positions the turret can reach
    // for that alliance
    public enum TurretAngles {
        BLUE_ZERO_POSITION(12200),
        BLUE_LOWER_45DEG(12100),
        BLUE_90DEG(9870),
        BLUE_135DEG(5875),
        BLUE_180DEG(6780),
        BLUE_225DEG(2680),
        BLUE_270DEG(650),
        BLUE_UPPER_45DEG(-200),
        BLUE_MAX_FROM_ZERO_POSE(0),

        // INTAKE FACING RIGHT, DEGS TOWARDS THE LEFT (counterclockwise)
        RED_ZERO_POSITION(0),
        RED_LOWER_45DEG(-200),
        RED_90DEG(650),
        RED_135DEG(2680),
        RED_180DEG(3760),
        RED_225DEG(2680),
        RED_270DEG(6780),
        RED_315DEG(5875),
        RED_360DEG(9870),
        RED_UPPER_45DEG(12100),
        RED_MAX_FROM_ZERO_POSE(12200);

        public int position;
        TurretAngles(int position) {
            this.position = position;
        }

        public int getPosition() {
            return this.position;
        }
    }
    // ===================== CONTROLLER =====================
    private PIDController controller;
    public static double p = 0.00075, i = 0, d = 0.00003;
    public static double f = 0.0065;

    public static int target = 0;

    // ===================== DEVICES =====================
    private final DcMotorEx turret;
    private final DigitalChannel magneticLimitSwitch;

    // ===================== STATES =====================
    public static boolean enabled = false;
    public static boolean homed = false;

    // ===================== Turret geometry / gearing =====================
    public static double MOTOR_REVS_PER_TURRET_REV = 3.0;
    public static double TICKS_PER_MOTOR_REV = 4096.0; // MELONBOTICS CPR
    public static int UPPER_LIMIT = 12200; // INCREMENTAL UPPER LIMIT (WIRES ON RIGHT)
    public static int LOWER_LIMIT = 0; // INCREMENTAL LOWER LIMIT (WIRES ON LEFT)
    public static int STARTING_OFFSET = 0;

    public MotorTurretTracker(HardwareMap hardwareMap, boolean isRed) {
        turret = hardwareMap.get(DcMotorEx.class, "turret");
        magneticLimitSwitch = hardwareMap.get(DigitalChannel.class, "magLimit");

        turret.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        turret.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        turret.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        magneticLimitSwitch.setMode(DigitalChannel.Mode.INPUT);

        if (!isRed) {
            STARTING_OFFSET = UPPER_LIMIT;
        }

        controller = new PIDController(p, i, d);
    }

    @Override
    public void periodic() {
        if (enabled) {
            controller.setPID(p, i, d);
            int currentPos = turret.getCurrentPosition();
            double pid = controller.calculate(currentPos, target);

            double ff = Math.cos(Math.toRadians(target)) * f;

            double power = pid + ff;

            turret.setPower(-power);
        }
    }

    public boolean isAtLimit() {
        return !magneticLimitSwitch.getState();
    }

    public int getCurrentEncoderPos() {
        return turret.getCurrentPosition();
    }

    // Adds starting offset as red side starts at absolute 0 and blue starts at absolute max
    public int getTarget() {
        return target + STARTING_OFFSET;
    }

    public void setTarget(int target) {
        MotorTurretTracker.target = target;
    }

    public void reset() {
        turret.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        setTarget(0);
    }

    public double getCurrent() {
        return turret.getCurrent(CurrentUnit.AMPS);
    }
}
