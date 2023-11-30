package org.firstinspires.ftc.teamcode.mechanisms;

import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Drivetrain {
    private PIDController fLController;
    private PIDController fRController;
    private PIDController bLController;
    private PIDController bRController;

    public  double p = 0.006, i = 0, d = 0;
    public  double f = 0.0;

    private   int target = 0;

    private DcMotorEx front_left;
    private DcMotorEx back_left;
    private DcMotorEx back_right;
    private DcMotorEx front_right;

    public Drivetrain(HardwareMap hwMap){
        front_left = hwMap.get(DcMotorEx.class, "front_left");
        back_left = hwMap.get(DcMotorEx.class, "back_left");
        back_right = hwMap.get(DcMotorEx.class, "back_right");
        front_right = hwMap.get(DcMotorEx.class, "front_right");

        front_left.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        back_left.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        back_right.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        front_right.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        front_left.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        back_left.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        back_right.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        front_right.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        back_left.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        back_right.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        front_left.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        front_right.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        front_left.setDirection(DcMotorSimple.Direction.REVERSE);
        back_left.setDirection(DcMotorSimple.Direction.REVERSE);
        back_right.setDirection(DcMotorSimple.Direction.FORWARD);
        front_right.setDirection(DcMotorSimple.Direction.FORWARD);

        fLController = new PIDController(p, i, d);
        fRController = new PIDController(p, i, d);
        bLController = new PIDController(p, i, d);
        bRController = new PIDController(p, i, d);
    }
    public void setTarget(int t){
        target += t;
    }
    public int getTarget(){
        return target;
    }
    public void update(){
        fLController.setPID(p - 0.002, i, d);
        fRController.setPID(p, i, d);
        bLController.setPID(p - 0.0015, i, d);
        bRController.setPID(p + 0.002, i, d);
        int slidePosFL = front_left.getCurrentPosition();
        int slidePosFR = front_right.getCurrentPosition();
        int slidePosBL = back_left.getCurrentPosition();
        int slidePosBR = back_right.getCurrentPosition();
        double pidFL = fLController.calculate(slidePosFL, -target);
        double pidFR = fRController.calculate(slidePosFR, -target);
        double pidBL = bLController.calculate(slidePosBL, target);
        double pidBR = bRController.calculate(slidePosBR, target);

        double powerFL = pidFL + f;
        double powerFR = pidFR + f;
        double powerBL = pidBL + f;
        double powerBR = pidBR + f;

        front_left.setPower(-0.6 * powerFL);
        front_right.setPower(-0.6 * powerFR);
        back_left.setPower(0.6 * powerBL);
        back_right.setPower(0.6 * powerBR);
    }
    public void idle(){
        front_left.setPower(0);
        front_right.setPower(0);
        back_left.setPower(0);
        back_right.setPower(0);
    }
    public double positionDifference(){
        return Math.abs(Math.abs(front_left.getCurrentPosition()) - Math.abs(target));
    }
    public void stopReset(){
        front_left.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        back_left.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        back_right.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        front_right.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        front_left.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        back_left.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        back_right.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        front_right.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        target = 0;
    }
    public void setDrivePowers(double power){
        front_left.setPower(power);
        front_right.setPower(power);
        back_left.setPower(power);
        back_right.setPower(power);
    }
}