
package org.firstinspires.ftc.teamcode.opmodes;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "SlideValues")
@Config
@Disabled
public class SlideValues extends LinearOpMode {
    private PIDController controller;
    private DcMotorEx slide_left;
    private DcMotorEx slide_right;

    private Servo arm_left;
    private Servo arm_right;

    public static double p = 0, i = 0, d = 0;
    public static double f = 0;
    public static int target = 0;


    @Override
    public void runOpMode() {

        arm_left = hardwareMap.get(Servo.class, "arm_left");
        arm_right = hardwareMap.get(Servo.class, "arm_right");
        arm_left.setDirection(Servo.Direction.REVERSE);

        slide_left = hardwareMap.get(DcMotorEx.class, "slide_left");
        slide_right = hardwareMap.get(DcMotorEx.class, "slide_right");
        slide_left.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        slide_right.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        slide_left.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        slide_right.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);

        final double INTAKE_POSITION = 0.545;
        //arm_left.setPosition(INTAKE_POSITION);
        //arm_right.setPosition(INTAKE_POSITION);
        controller = new PIDController(p, i, d);
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        waitForStart(); //1100, 420, 900 for slides

        while (opModeIsActive() && !isStopRequested()) {
            int leftPos = slide_left.getCurrentPosition();
            int rightPos = slide_right.getCurrentPosition();

            telemetry.addData("leftPos: ",leftPos);
            telemetry.addData("rightPos: ", rightPos);
            telemetry.update();
        }

    }


    /* Update the telemetry */


}