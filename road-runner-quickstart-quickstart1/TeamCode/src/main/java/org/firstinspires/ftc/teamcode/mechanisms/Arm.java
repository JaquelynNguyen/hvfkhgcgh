package org.firstinspires.ftc.teamcode.mechanisms;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

public class Arm {

    private Servo arm_left;
    private Servo arm_right;
    private CRServo dropbox;
    public static double position;
    ElapsedTime timer;
    boolean dropping = false;


    public static class Presets {
        public static final double INTAKE_POSITION = 0.59;
        public static final double OUTTAKE_POSITION = 0.2;
    }

    public Arm(HardwareMap hardwareMap) {
        arm_left = hardwareMap.get(Servo.class, "arm_left");
        arm_right = hardwareMap.get(Servo.class, "arm_right");
        arm_left.setDirection(Servo.Direction.REVERSE);
        dropbox = hardwareMap.get(CRServo.class, "dropbox");
        timer = new ElapsedTime();

    }

    public void setPosition(double t) {
        arm_left.setPosition(t);
        arm_right.setPosition(t);
    }

    public void setDropping(boolean b) {
        dropping = b;
    }
    public void update(){
        if(dropping) {
            dropbox.setPower(-1);
        } else {
            dropbox.setPower(0);
        }
    }
}
