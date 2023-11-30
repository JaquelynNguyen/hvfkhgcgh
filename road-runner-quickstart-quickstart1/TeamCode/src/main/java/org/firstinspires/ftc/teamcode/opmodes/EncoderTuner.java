 
package org.firstinspires.ftc.teamcode.opmodes;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "EncoderTuner")
@Config
//@Disabled
public class EncoderTuner extends LinearOpMode {
    public enum State {
        FL,
        FR,
        BL,
        BR

    };
    private PIDController controllerFL;
    private PIDController controllerFR;
    private PIDController controllerBL;
    private PIDController controllerBR;
    private DcMotorEx slide_left;
    private DcMotorEx slide_right;
    private DcMotorEx front_left;
    private DcMotorEx front_right;
    private DcMotorEx back_left;
    private DcMotorEx back_right;

    private Servo arm_left;
    private Servo arm_right;

    public static double p = 0.009, i = 0, d = 0;
    //0.009 fl bl? br
    //0.01
    //fr = bl?
    //br = fl?
    public static double f = 0;
    public static int target = 0;


    @Override
    public void runOpMode() {

        arm_left = hardwareMap.get(Servo.class, "arm_left");
        arm_right = hardwareMap.get(Servo.class, "arm_right");
        arm_left.setDirection(Servo.Direction.REVERSE);

        slide_left = hardwareMap.get(DcMotorEx.class, "slide_left");
        slide_right = hardwareMap.get(DcMotorEx.class, "slide_right");
        slide_right.setDirection(DcMotorSimple.Direction.REVERSE);
        slide_left.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        slide_right.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        slide_left.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        slide_right.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);

        front_left = hardwareMap.get(DcMotorEx.class, "front_left");
        front_right = hardwareMap.get(DcMotorEx.class, "front_right");
        back_left = hardwareMap.get(DcMotorEx.class, "back_left");
        back_right = hardwareMap.get(DcMotorEx.class, "back_right");
        front_left.setDirection(DcMotorSimple.Direction.REVERSE);
        front_left.setDirection(DcMotorSimple.Direction.FORWARD);
        front_right.setDirection(DcMotorSimple.Direction.REVERSE);
        front_left.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        front_right.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        back_left.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        back_right.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);

        front_left.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        front_right.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        back_left.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        back_right.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);

        controllerFL = new PIDController(p, i, d);
        controllerFR = new PIDController(p, i, d);
        controllerBL = new PIDController(p, i, d);
        controllerBR = new PIDController(p, i, d);
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        waitForStart(); //1100, 420, 900 for slides

        while (opModeIsActive() && !isStopRequested()) {
                controllerFL.setPID(p, i, d);
                controllerFR.setPID(p, i, d);
                controllerBL.setPID(p, i, d);
                controllerBR.setPID(p, i, d);
                int posFL = front_left.getCurrentPosition();
                int posFR = front_left.getCurrentPosition();
                int posBL = front_left.getCurrentPosition();
                int posBR = front_left.getCurrentPosition();

                double pidFL = controllerFL.calculate(posFL, target);
                double powerFL = pidFL + f;
            double pidFR = controllerFR.calculate(posFR, target);
            double powerFR = pidFR + f;
            double pidBL = controllerBL.calculate(posBL, target);
            double powerBL = pidBL + f;
            double pidBR = controllerFL.calculate(posBR, target);
            double powerBR = pidBR + f;

                front_left.setPower(powerFL);
                //front_right.setPower(powerFL);
                back_left.setPower(powerFL);
                //back_right.setPower(powerFL);
                //telemetry.addData("posFL: ",posFL);
            telemetry.addData("posFR: ",posFR);
            telemetry.addData("posBL: ",posBL);
            telemetry.addData("posBR: ",posBR);
                telemetry.addData("target", target);
                telemetry.update();
            }

        }


    /* Update the telemetry */


}