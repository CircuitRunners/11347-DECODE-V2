package org.firstinspires.ftc.teamcode.testing;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.subsystems.intake.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.support.OdoAbsoluteHeadingTracking;

@TeleOp
public class OdoAbsoluteTrackingTester extends CommandOpMode {
    private IntakeSubsystem intake;
    private OdoAbsoluteHeadingTracking odoHeading;
    private GoBildaPinpointDriver pinpoint;

    @Override
    public void initialize() {
        intake = new IntakeSubsystem(hardwareMap, telemetry);

        odoHeading = new OdoAbsoluteHeadingTracking(
                intake.leftOdoMotor(),
                intake.rightOdoMotor()
        );

        pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, "pinpoint");
        configurePinpoint();

        odoHeading.reset(0.0);
    }

    @Override
    public void run() {
        super.run();
        odoHeading.update();
        pinpoint.update();

        telemetry.addData("Heading (deg)", odoHeading.getHeadingDeg());
        telemetry.addData("Heading (rad)", odoHeading.getHeadingRad());

        telemetry.addData("Pinpoint Heading", pinpoint.getHeading(AngleUnit.DEGREES));
        telemetry.update();
    }

    private void configurePinpoint() {
        pinpoint.resetPosAndIMU();
        pinpoint.setOffsets(145.12237, -145.12237, DistanceUnit.MM);
        pinpoint.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);

        pinpoint.setEncoderDirections(
                GoBildaPinpointDriver.EncoderDirection.REVERSED,
                GoBildaPinpointDriver.EncoderDirection.FORWARD
        );
    }
}
