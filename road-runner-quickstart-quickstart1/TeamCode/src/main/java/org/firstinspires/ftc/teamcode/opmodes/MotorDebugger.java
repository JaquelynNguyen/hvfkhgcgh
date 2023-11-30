package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name = "MotorDebugger")
@Disabled
public class MotorDebugger extends LinearOpMode {



    private DcMotor front_left;
    private DcMotor back_left;
    private DcMotor back_right;
    private DcMotor front_right;

    /**
     * This function is executed when this Op Mode is selected from the Driver Station.
     */
    @Override
    public void runOpMode() {
        front_left = hardwareMap.get(DcMotor.class, "front_left");
        back_left = hardwareMap.get(DcMotor.class, "back_left");
        back_right = hardwareMap.get(DcMotor.class, "back_right");
        front_right = hardwareMap.get(DcMotor.class, "front_right");


        // Put initialization blocks here.
        front_left.setDirection(DcMotorSimple.Direction.REVERSE);
        back_left.setDirection(DcMotorSimple.Direction.REVERSE);



        waitForStart();

        if (opModeIsActive()) {
            // Put run blocks here.
            telemetry.addData("Status", "Initiallized");
            while (opModeIsActive()) {

                if (gamepad2.dpad_left) {
                    front_left.setPower(1);
                } else {
                    front_left.setPower(0);
                }

                if (gamepad2.dpad_right) {
                    front_right.setPower(1);
                } else {
                    front_right.setPower(0);
                }

                if (gamepad2.square) {
                    back_left.setPower(1);
                } else {
                    back_left.setPower(0);
                }

                if (gamepad2.circle) {
                    back_right.setPower(1);
                } else {
                    back_right.setPower(0);
                }
            }
        }
        telemetry.update();
    }



    private void setDrivePowers(double bLPower, double bRPower, double fLPower, double fRPower) {
        double maxSpeed = 1.0;
        maxSpeed = Math.max(maxSpeed, Math.abs(bLPower));
        maxSpeed = Math.max(maxSpeed, Math.abs(bRPower));
        maxSpeed = Math.max(maxSpeed, Math.abs(fLPower));
        maxSpeed = Math.max(maxSpeed, Math.abs(fRPower));

        bLPower /= maxSpeed;
        bRPower /= maxSpeed;
        fLPower /= maxSpeed;
        fRPower /= maxSpeed;

        back_left.setPower(bLPower);
        back_right.setPower(bRPower);
        front_left.setPower(fLPower);
        front_right.setPower(fRPower);
    }

}