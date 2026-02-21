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
    private CRServo intakeRed, intakeBlue;
    public static boolean reverseBlueServo = false; // true
    public static boolean reverseRedServo = false;

    public IntakeSubsystem(HardwareMap hardwareMap, Telemetry telemetry) {
        intakeChubM = hardwareMap.get(DcMotorEx.class, "intakeChub");
        intakeEhubM = hardwareMap.get(DcMotorEx.class, "intakeEhub");

        intakeRed = new CRServo(hardwareMap, "intakeRed");
        intakeBlue = new CRServo(hardwareMap, "intakeBlue");

        intakeChubM.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        intakeEhubM.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);

        telemetry.addLine("Intake Init Done");
    }

    public void intakeChub(double power) {
        intakeChubM.setPower(Range.clip(power, -0.9, 0.9));

        double sPower = reverseBlueServo ? -power : power;
        intakeBlue.set(sPower);
    }

    public void intakeEhub(double power) {
        intakeEhubM.setPower(Range.clip(power, -0.9, 0.9));

        double sPower = reverseRedServo ? -power : power;
        intakeRed.set(sPower);
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
