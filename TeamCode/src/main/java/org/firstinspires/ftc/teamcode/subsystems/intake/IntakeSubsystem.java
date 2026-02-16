package org.firstinspires.ftc.teamcode.subsystems.intake;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.hardware.motors.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;

@Config
public class IntakeSubsystem extends SubsystemBase {
    private DcMotorEx intakeChubM, intakeEhubM;
    private CRServo intakeChubS, intakeEhubS;
    public static boolean reverseChubServoDirection = false; // true
    public static boolean reverseEhubServoDirection = false;

    public IntakeSubsystem(HardwareMap hardwareMap, Telemetry telemetry) {
        intakeChubM = hardwareMap.get(DcMotorEx.class, "intakeChub");
        intakeEhubM = hardwareMap.get(DcMotorEx.class, "intakeEhub");

        intakeChubS = new CRServo(hardwareMap, "intakeChubServo");
        intakeEhubS = new CRServo(hardwareMap, "intakeEhubServo");

        intakeChubM.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        intakeEhubM.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);

        telemetry.addLine("Intake Init Done");
    }

    public void intakeChub(double power) {
        intakeChubM.setPower(Range.clip(power, -0.9, 0.9));

        double sPower = reverseChubServoDirection ? -power : power;
        intakeChubS.set(sPower);
    }

    public void intakeEhub(double power) {
        intakeEhubM.setPower(Range.clip(power, -0.9, 0.9));

        double sPower = reverseEhubServoDirection ? -power : power;
        intakeEhubS.set(sPower);
    }

    public void intake(double power) {
        intakeChub(power);
        intakeEhub(power);
    }

    public void stop() {
        intake(0);
    }

    public DcMotorEx rightOdoMotor() {
        return intakeChubM;
    }

    public DcMotorEx leftOdoMotor() {
        return intakeEhubM;
    }
}
