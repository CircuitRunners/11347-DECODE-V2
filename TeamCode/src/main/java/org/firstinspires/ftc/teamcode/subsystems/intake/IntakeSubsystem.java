package org.firstinspires.ftc.teamcode.subsystems.intake;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class IntakeSubsystem extends SubsystemBase {
    private DcMotorEx intakeChub, intakeEhub;

    public IntakeSubsystem(HardwareMap hardwareMap, Telemetry telemetry) {
        intakeChub = hardwareMap.get(DcMotorEx.class, "intakeChub");
        intakeEhub = hardwareMap.get(DcMotorEx.class, "intakeEhub");

        telemetry.addLine("Intake Init Done");
    }

    public void intakeChub(double power) {
        intakeChub.setPower(Range.clip(power, -0.8, 0.8));
    }

    public void intakeEhub(double power) {
        intakeEhub.setPower(Range.clip(power, -0.8, 0.8));
    }

    public void intake(double power) {
        intakeChub.setPower(power);
        intakeEhub.setPower(power);
    }

    public void stop() {
        intake(0);
    }
}
