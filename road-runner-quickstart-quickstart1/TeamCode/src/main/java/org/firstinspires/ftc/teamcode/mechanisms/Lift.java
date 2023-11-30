package org.firstinspires.ftc.teamcode.mechanisms;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Lift {

    private PIDController controller;
    private DcMotorEx slide_left;
    private DcMotorEx slide_right;

    public static double p = 0.01, i = 0, d = 0;
    public static double f = 0;
    public static int target = 0;

    public static class Presets {
        public static final int IDLE = 0;
        public static final int LOW_DROP = 1100;
    }

    public Lift(HardwareMap hardwareMap) {
        slide_left = hardwareMap.get(DcMotorEx.class, "slide_left");
        slide_right = hardwareMap.get(DcMotorEx.class, "slide_right");
        slide_right.setDirection(DcMotorSimple.Direction.REVERSE);
        slide_left.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        slide_right.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        slide_left.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        slide_right.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);

        controller = new PIDController(p, i, d);
    }

    public void setTarget(int t){
        target = t;
    }
    public int getTarget(){
        return target;
    }
    public void update(){
        controller.setPID(p, i, d);
        int slidePosL = slide_left.getCurrentPosition();
        double pid = controller.calculate(slidePosL, target);

        double power = pid + f;

        slide_left.setPower(-power);
        slide_right.setPower(-power);
    }
    public void idle(){
        slide_right.setPower(0.04);
        slide_left.setPower(0.04);
    }
    public void stopReset(){
        slide_left.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        slide_right.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        slide_left.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        slide_right.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
    }
    public void setLiftPowers(double power){
        slide_left.setPower(-power);
        slide_right.setPower(-power);
    }
}
