package org.firstinspires.ftc.teamcode.subsystems.transfer;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;


@Config
public class kicker extends SubsystemBase {
    private Servo kickerServo1, kickerServo2, kickerServo3;//, turretServo2;


//    public enum kickerState {
//        UP(0.25), //not right values
//        DOWN(0.00);
//
//        public final double position;
//        kickerState(double position) {
//            this.position = position;
//        }
//
//        public double getPosition() {
//            return position;
//        }
//    }
    private final double up = 0.23; //1:0.22 //2:0.23 /3:0.23
    private final double down = 0.52; //0.55 old value






    public kicker(HardwareMap hardwareMap) {
        kickerServo1 = hardwareMap.get(Servo.class, "ks1");
        kickerServo2 = hardwareMap.get(Servo.class, "ks2");
        kickerServo3 = hardwareMap.get(Servo.class, "ks3");

    }

    public void setKickerServo1(double position){
        kickerServo1.setPosition(position);
    }
    public void setKickerServo1Up() {
        kickerServo1.setPosition(up);
    }

    public void setKickerServo2Up() {
        kickerServo2.setPosition(up);
    }
    public void setKickerServo3Up() {
        kickerServo3.setPosition(up);
    }

    public void setKickerServo1Down() {
        kickerServo1.setPosition(down);
    }

    public void setKickerServo2Down() {
        kickerServo2.setPosition(down);
    }
    public void setKickerServo3Down() {
        kickerServo3.setPosition(down);
    }

    public void kickerTransferUP(int servoNumber){
        switch (servoNumber) {
            case 0:
                setKickerServo1Up();
                servoNumber = 1;
                break;
            case 1:
                setKickerServo2Up();

                break;
            case 2:
                setKickerServo3Up();
                break;
            default:
                break;
        }
    }


    public void kickerTransferDOWN(int servoNumber){
        switch (servoNumber) {
            case 0:
                setKickerServo1Down();
                break;
            case 1:
                setKickerServo2Down();
                break;
            case 2:
                setKickerServo3Down();
                break;
            default:
                break;
        }
    }

    public void kickerDownPosition(){
        setKickerServo1Down();
        setKickerServo2Down();
        setKickerServo3Down();
    }

}
