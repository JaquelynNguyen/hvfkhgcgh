package org.firstinspires.ftc.teamcode.opmodes;

import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.arcrobotics.ftclib.gamepad.ToggleButtonReader;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name = "ActualTeleOp")
//@Disabled
public class ActualTeleOp extends LinearOpMode {

    public enum V4bState {
        V4B_START,
        V4B_800,

    };

    V4bState v4bState = V4bState.V4B_START;
    private DcMotor front_left;
    private DcMotor back_left;
    private DcMotor back_right;
    private DcMotor front_right;


    private Servo arm_left;
    private Servo arm_right;
    private Servo shooterServo;

    private DcMotor slide_left;
    private DcMotor slide_right;
    private DcMotor intakeMotor;
    private Servo rightFoldServo;
    private Servo leftFoldServo;
    private CRServo dropbox;
    private DcMotor climbMotor;
    //private CRServo tapeServo;
    private IMU imu;
    ElapsedTime intakeTimer;

    ElapsedTime v4bTimer = new ElapsedTime();
    /**
     * This function is executed when this Op Mode is selected from the Driver Station.
     */
    @Override
    public void runOpMode() {
        front_left = hardwareMap.get(DcMotor.class, "front_left");
        back_left = hardwareMap.get(DcMotor.class, "back_left");
        back_right = hardwareMap.get(DcMotor.class, "back_right");
        front_right = hardwareMap.get(DcMotor.class, "front_right");

        shooterServo = hardwareMap.get(Servo.class, "shooter");

        //tapeServo = hardwareMap.get(CRServo.class, "tapeServo");
        slide_left = hardwareMap.get(DcMotor.class, "slide_left");
        slide_right = hardwareMap.get(DcMotor.class, "slide_right");

        intakeMotor = hardwareMap.get(DcMotor.class, "intakeMotor");

        slide_left.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        slide_right.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        slide_left.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        slide_right.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        /*
        rightFoldServo = hardwareMap.get(Servo.class, "rightFoldServo");
        leftFoldServo = hardwareMap.get(Servo.class, "leftFoldServo");

 */

        arm_left = hardwareMap.get(Servo.class, "arm_left");
        arm_right = hardwareMap.get(Servo.class, "arm_right");
        arm_left.setDirection(Servo.Direction.REVERSE);
        dropbox = hardwareMap.get(CRServo.class, "dropbox");


        // Put initialization blocks here.
        front_left.setDirection(DcMotorSimple.Direction.REVERSE);
        back_left.setDirection(DcMotorSimple.Direction.REVERSE);


        //button toggle thing from FTCLib
        GamepadEx opController = new GamepadEx(gamepad2);
        ToggleButtonReader dropboxReader = new ToggleButtonReader(
                opController, GamepadKeys.Button.X
        );
        ToggleButtonReader turnReader = new ToggleButtonReader(
                opController, GamepadKeys.Button.DPAD_RIGHT
        );
        ToggleButtonReader climbReader = new ToggleButtonReader(
                opController, GamepadKeys.Button.DPAD_UP
        );

        GamepadEx driveController = new GamepadEx(gamepad1);
        ToggleButtonReader intakeReader = new ToggleButtonReader(
                driveController, GamepadKeys.Button.X
        );

        ToggleButtonReader outtakeReader = new ToggleButtonReader(
                driveController, GamepadKeys.Button.B
        );

        /*
        ButtonReader intakeReader = new ButtonReader(
                driveController, GamepadKeys.Button.X
        );

        ButtonReader outtakeReader = new ButtonReader(
                driveController, GamepadKeys.Button.B
        );

         */


        imu = hardwareMap.get(IMU.class, "imu");
        imu.initialize(new IMU.Parameters(new RevHubOrientationOnRobot(RevHubOrientationOnRobot.LogoFacingDirection.RIGHT, RevHubOrientationOnRobot.UsbFacingDirection.UP)));
        //clawopen checks if it's being used to open or close
        //true = open
        //false = close
        boolean clawopen = false;
        boolean climbing = false;
        final double INTAKE_POSITION = 0.6;
        final double OUTTAKE_POSITION = 0.2;
        boolean intakeState = false;
        boolean outtakeState = false;
        //false = back, true = front
        boolean frontOrBack = false;

        intakeTimer = new ElapsedTime();
        boolean timerStart = false;
        boolean in = true;

        arm_left.setPosition(INTAKE_POSITION);
        arm_right.setPosition(INTAKE_POSITION);
        shooterServo.setPosition(1);
        boolean spinnyIn = false;
        v4bState = V4bState.V4B_START;
        waitForStart();
        v4bTimer.reset();
        if (opModeIsActive()) {

            // Put run blocks here.
            telemetry.addData("Status", "Initiallized");
            while (opModeIsActive()) {
                double forward = 1 * gamepad1.left_stick_y;
                double strafe = -1 * gamepad1.right_stick_x;
                double rotate = 1 * 0.7 * gamepad1.left_stick_x;
                double slides = -gamepad2.left_stick_y;
                //double climb = -gamepad2.right_stick_y;

                double bLPower = forward - strafe + rotate; //
                double bRPower = forward + strafe - rotate; //
                double fLPower = forward + strafe + rotate; //
                double fRPower = forward - strafe - rotate; // - strafe
                setDrivePowers(bLPower, bRPower, fLPower, fRPower);
                // reset speed variables

                //Shooter code?? todo: set shooter servos to shooting position
                //dpad up = shoot
                if (gamepad1.dpad_up) {
                    shooterServo.setPosition(-0.3);
                }





                /*
                //climber code
                //right joystick up and down
                if (gamepad1.right_trigger > 0.2) {
                    climbMotor.setPower(.6);
                } else if (gamepad1.left_trigger > 0.2) {
                    climbMotor.setPower(-.6);
                } else {
                    climbMotor.setPower(0);
                }

                 */

                //climber
                if (gamepad2.dpad_up) {
                    climbing = true;
                }
                if (gamepad2.dpad_down) {
                    climbing = false;
                }


/*
                switch(v4bState) {
                    case V4B_START:
                        if(gamepad2.a) {
                            arm_left.setPosition(INTAKE_POSITION);
                            arm_right.setPosition(INTAKE_POSITION);
                        }
                         else if (slide_left.getCurrentPosition() >= 600) {
                            arm_left.setPosition(OUTTAKE_POSITION);
                            arm_right.setPosition(OUTTAKE_POSITION);
                            v4bState = V4bState.V4B_800;
                        }
                        break;
                    case V4B_800:
                        if (gamepad2.a) {
                            arm_left.setPosition(INTAKE_POSITION);
                            arm_right.setPosition(INTAKE_POSITION);
                            v4bState = V4bState.V4B_START;
                        }
                        break;
                }

 */
                /*
                if(gamepad2.a) {
                    arm_left.setPosition(INTAKE_POSITION);
                    arm_right.setPosition(INTAKE_POSITION);
                }
                else if (slide_left.getCurrentPosition() > 950 && !in) {
                    arm_left.setPosition(OUTTAKE_POSITION);
                    arm_right.setPosition(OUTTAKE_POSITION);
                }

                 */

                //slide code
                if (-gamepad2.left_stick_y < -0.2) {
                    slide_left.setPower(0.9);
                    slide_right.setPower(-0.9);
                } else if (-gamepad2.left_stick_y > 0.2) {
                    slide_left.setPower(-1);
                    slide_right.setPower(1);
                } else if (climbing) {
                    slide_left.setPower(.8);
                    slide_right.setPower(-.8);
                } else {
                    slide_left.setPower(0);
                    slide_right.setPower(0);
                }

                /*
                 */
/*

                //intake code
                if(intakeReader.wasJustReleased()) {
                    intakeState = !intakeState;
                }
                if (outtakeReader.wasJustReleased()) {
                    outtakeState = !outtakeState;
                }

                if(intakeState) {
                    intakeMotor.setPower(0.7);
                    outtakeState = false;
                }
                if (outtakeState) {
                    intakeMotor.setPower(-0.7);
                    intakeState = false;
                }
                if(!intakeState && !outtakeState){
                    intakeMotor.setPower(0);
                }

 */
                if(gamepad1.left_trigger > 0.2) {
                    intakeMotor.setPower(1);
                    dropbox.setPower(1);
                } else if (gamepad1.right_trigger > 0.2) {
                    intakeMotor.setPower(-1);
                } else {
                    intakeMotor.setPower(0);
                    if (-gamepad2.right_stick_y < -0.2) {
                        dropbox.setPower(-1);
                    }
                    else {
                        dropbox.setPower(0);
                    }
                }
                /*
                if(intakeReader.getState()) {
                    timerStart = false;
                }
                if(-gamepad2.right_stick_y < -0.2 && !timerStart) {
                    dropbox.setPower(-1);
                } else if(!timerStart) {
                    dropbox.setPower(0);
                    intakeMotor.setPower(0);
                }
                if(!intakeReader.getState()) {
                    intakeTimer.reset();
                    timerStart = true;

                }
                if (outtakeReader.getState() && !timerStart) {
                    intakeMotor.setPower(-0.7);
                } else if (timerStart) {
                    intakeMotor.setPower(0.7);
                    dropbox.setPower(1);
                    if(intakeTimer.seconds() > 10) {
                        timerStart = false;

                    }
                }

                 */



                /*
                if(turnReader.getState()) {
                    v4bTurn.setPosition(dropAngleIntake);
                } else {
                    v4bTurn.setPosition(dropAngleOut);
                }

                 */


//-129262.54 92 5/16
                if(gamepad2.y) {
                    arm_left.setPosition(OUTTAKE_POSITION);
                    arm_right.setPosition(OUTTAKE_POSITION);
                } else if (gamepad2.a) {
                    arm_left.setPosition(INTAKE_POSITION);
                    arm_right.setPosition(INTAKE_POSITION);
                }

                /*

        imu = hardwareMap.get(IMU.class, "imu");
        imu.initialize(new IMU.Parameters(new RevHubOrientationOnRobot(RevHubOrientationOnRobot.LogoFacingDirection.LEFT, RevHubOrientationOnRobot.UsbFacingDirection.UP)));

                double forward = 0.85 * gamepad1.left_stick_y;
                double strafe = -0.85 * gamepad1.left_stick_x;
                double rotate = 0.85 * gamepad1.right_stick_x;
                double slides = -gamepad2.left_stick_y;
                //double climb = -gamepad2.right_stick_y;

                //angle reset
                if (gamepad1.options) {
                    imu.resetYaw();
                }

                double botHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);

                // Rotate the movement direction counter to the bot's rotation
                double rotX = strafe * Math.cos(-botHeading) - forward * Math.sin(-botHeading);
                double rotY = strafe * Math.sin(-botHeading) + forward * Math.cos(-botHeading);

                // Denominator is the largest motor power (absolute value) or 1
                // This ensures all the powers maintain the same ratio,
                // but only if at least one is out of the range [-1, 1]
                double denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(rotate), 1);
                double frontLeftPower = (rotY + rotX + rotate) / denominator;
                double backLeftPower = (rotY - rotX + rotate) / denominator;
                double frontRightPower = (rotY - rotX - rotate) / denominator;
                double backRightPower = (rotY + rotX - rotate) / denominator;


                front_left.setPower(frontLeftPower);
                back_left.setPower(backLeftPower);
                front_right.setPower(frontRightPower);
                back_right.setPower(backRightPower);
                 */







                // set motor parameters to driver station
                telemetry.addData("left slide pos: ", slide_left.getCurrentPosition());
                telemetry.addData("right slide pos: ", slide_right.getCurrentPosition());
                telemetry.addData("left pos: ",clawopen);
                telemetry.addData("forward: ", forward);
                telemetry.addData("strafe: ", strafe);
                telemetry.addData("rotate: ", rotate);
                telemetry.update();

                dropboxReader.readValue();
                intakeReader.readValue();
                turnReader.readValue();
                outtakeReader.readValue();
                //driveController.readButtons();
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