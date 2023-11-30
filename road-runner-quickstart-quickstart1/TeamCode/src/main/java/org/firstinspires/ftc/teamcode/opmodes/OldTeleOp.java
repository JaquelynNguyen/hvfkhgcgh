package org.firstinspires.ftc.teamcode.opmodes;

import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.arcrobotics.ftclib.gamepad.ToggleButtonReader;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name = "Meet0TeleOp")
@Disabled
public class OldTeleOp extends LinearOpMode {

    public enum V4bState {
        V4B_START,
        V4B_MID,
        V4B_WRISTVERT,
        V4B_MOVE, //sadfasdf
        V4B_WRISTHORZ
    };

    V4bState v4bState = V4bState.V4B_START;
    private DcMotor front_left;
    private DcMotor back_left;
    private DcMotor back_right;
    private DcMotor front_right;

     private Servo wrist;
     private Servo leftClaw;
     private Servo rightClaw;

     private CRServo arm_left;
     private CRServo arm_right;
     private CRServo v4bTurn;

    private Servo shooterServo;

    private DcMotor slide_left;
    private DcMotor slide_right;
    private DcMotor intakeMotor;
    private Servo rightFoldServo;
    private Servo leftFoldServo;
    private Servo dropbox;
    private DcMotor climbMotor;
    //private CRServo tapeServo;
    private IMU imu;

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

        //shooterServo = hardwareMap.get(Servo.class, "shooterServo");

        //tapeServo = hardwareMap.get(CRServo.class, "tapeServo");
        slide_left = hardwareMap.get(DcMotor.class, "slide_left");
        slide_right = hardwareMap.get(DcMotor.class, "slide_right");

        intakeMotor = hardwareMap.get(DcMotor.class, "intakeMotor");
        rightFoldServo = hardwareMap.get(Servo.class, "rightFoldServo");
        leftFoldServo = hardwareMap.get(Servo.class, "leftFoldServo");

        arm_left = hardwareMap.get(CRServo.class, "arm_left");
        arm_right = hardwareMap.get(CRServo.class, "arm_right");
        v4bTurn = hardwareMap.get(CRServo.class, "v4bTurn");
        dropbox = hardwareMap.get(Servo.class, "dropbox");


        rightFoldServo.setDirection(Servo.Direction.REVERSE);

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

        GamepadEx driveController = new GamepadEx(gamepad1);
        ToggleButtonReader intakeReader = new ToggleButtonReader(
                driveController, GamepadKeys.Button.X
        );

        ToggleButtonReader outtakeReader = new ToggleButtonReader(
                driveController, GamepadKeys.Button.B
        );


        imu = hardwareMap.get(IMU.class, "imu");
        imu.initialize(new IMU.Parameters(new RevHubOrientationOnRobot(RevHubOrientationOnRobot.LogoFacingDirection.RIGHT, RevHubOrientationOnRobot.UsbFacingDirection.UP)));
        //clawopen checks if it's being used to open or close
        //true = open
        //false = close
        boolean clawopen = false;
        final double DROPBOX_CLOSED = 1;
        final double DROPBOX_OPEN = -.25;
        final double dropAngleIntake = 0.15;
        final double dropAngleOut = -.3;
        //false = back, true = front
        boolean frontOrBack = false;


        waitForStart();
        v4bTimer.reset();
        if (opModeIsActive()) {
            rightFoldServo.setPosition(.3);
            leftFoldServo.setPosition(.65);
            dropbox.setPosition(DROPBOX_CLOSED);
            // Put run blocks here.
            telemetry.addData("Status", "Initiallized");
            while (opModeIsActive()) {
                double forward = 0.85 * gamepad1.left_stick_y;
                double strafe = -0.85 * gamepad1.right_stick_x;
                double rotate = 0.85 * 0.7 * gamepad1.left_stick_x;
                double slides = -gamepad2.left_stick_y;
                //double climb = -gamepad2.right_stick_y;

                double bLPower = forward - strafe + rotate; //
                double bRPower = forward + strafe - rotate; //
                double fLPower = forward + strafe + rotate; //
                double fRPower = forward - strafe - rotate; // - strafe
                setDrivePowers(bLPower, bRPower, fLPower, fRPower);
                // reset speed variables

                //Shooter code?? todo: set shooter servos to shooting position
                //dpad left = shoot

                /*
                if (gamepad2.dpad_left) {
                    shooterServo.setPosition(1);
                }

                 */



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

                //slide code
                if (slides < -0.2) {
                    slide_left.setPower(0.9);
                    slide_right.setPower(-0.9);
                } else if (slides > 0.2) {
                    slide_left.setPower(-0.775);
                    slide_right.setPower(0.775);
                } else {
                    slide_left.setPower(0);
                    slide_right.setPower(0);
                }

                if(gamepad2.right_bumper) {
                    v4bTurn.setPower(0.15);
                } else if (gamepad2.left_bumper) {
                    v4bTurn.setPower(-0.15);
                } else {
                    v4bTurn.setPower(0);
                }

                //intake code
                //dpad right = on, otherwise off
                if(intakeReader.getState()) {
                    intakeMotor.setPower(0.7);
                } else if (outtakeReader.getState()) {
                    intakeMotor.setPower(-0.7);
                } else {
                    intakeMotor.setPower(0);
                }

                /*
                if(turnReader.getState()) {
                    v4bTurn.setPosition(dropAngleIntake);
                } else {
                    v4bTurn.setPosition(dropAngleOut);
                }

                 */

                if(-gamepad2.right_stick_y < -0.2) {
                    arm_left.setPower(-0.25);
                    arm_left.setPower(-0.25);
                    arm_right.setPower(0.25);
                } else if (-gamepad2.right_stick_y > 0.2) {
                    arm_left.setPower(0.25);
                    arm_right.setPower(-0.25);
                } else {
                    arm_left.setPower(0);
                    arm_right.setPower(0);
                }



                if(dropboxReader.getState()) {

                    dropbox.setPosition(DROPBOX_CLOSED);
                } else {
                    dropbox.setPosition(DROPBOX_OPEN);
                }



                // set motor parameters to driver station
                telemetry.addData("left slide pos: ", slide_left.getCurrentPosition());
                telemetry.addData("right slide pos: ", slide_right.getCurrentPosition());
                telemetry.addData("ClawOpen: ",clawopen);
                telemetry.addData("forward: ", forward);
                telemetry.addData("strafe: ", strafe);
                telemetry.addData("rotate: ", rotate);
                telemetry.update();

                dropboxReader.readValue();
                intakeReader.readValue();
                turnReader.readValue();
                outtakeReader.readValue();

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