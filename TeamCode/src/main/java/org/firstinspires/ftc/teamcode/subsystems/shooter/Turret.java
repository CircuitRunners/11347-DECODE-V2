package org.firstinspires.ftc.teamcode.subsystems.shooter;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;


@Config
public class Turret extends SubsystemBase {
    private Servo turretServo1;//, turretServo2;









    public Turret(HardwareMap hardwareMap) {
        turretServo1 = hardwareMap.get(Servo.class, "turretServo");
        //turretServo2 = hardwareMap.get(Servo.class, "rightDiffyServo");

    }


    public void setTurretPosition(double position) {
        turretServo1.setPosition(position);
        //turretServo2.setPosition(position);
    }


    public double getTurretPosition() {
        return turretServo1.getPosition()/2.0;
    }
}
