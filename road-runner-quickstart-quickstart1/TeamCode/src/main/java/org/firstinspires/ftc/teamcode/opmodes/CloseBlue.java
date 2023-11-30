package org.firstinspires.ftc.teamcode.opmodes;

import com.acmerobotics.roadrunner.geometry.Pose2d;

import org.firstinspires.ftc.teamcode.mechanisms.Arm;
import org.firstinspires.ftc.teamcode.mechanisms.Lift;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

/**
 * This opmode explains how you follow multiple trajectories in succession, asynchronously. This
 * allows you to run your own logic beside the drive.update() command. This enables one to run
 * their own loops in the background such as a PID controller for a lift. We can also continuously
 * write our pose to PoseStorage.
 * <p>
 * The use of a State enum and a currentState field constitutes a "finite state machine."
 * You should understand the basics of what a state machine is prior to reading this opmode. A good
 * explanation can be found here:
 * https://www.youtube.com/watch?v=Pu7PMN5NGkQ (A finite state machine introduction tailored to FTC)
 * or here:
 * https://gm0.org/en/stable/docs/software/finite-state-machines.html (gm0's article on FSM's)
 * <p>
 * You can expand upon the FSM concept and take advantage of command based programming, subsystems,
 * state charts (for cyclical and strongly enforced states), etc. There is still a lot to do
 * to supercharge your code. This can be much cleaner by abstracting many of these things. This
 * opmode only serves as an initial starting point.
 */
@Autonomous(name = "CloseBlue", group = "advanced")
@Disabled
public class CloseBlue extends LinearOpMode {
    private Servo claw;

    final double OPEN_CLAW = .46;
    final double CLOSED_CLAW = 0;

    // This enum defines our "state"
    // This is essentially just defines the possible steps our program will take
    enum State {
        START,
        TRAJECTORY_1L,
        TRAJECTORY_2L,
        TRAJECTORY_3L,
        TURN_1L,
        TRAJECTORY_4L,
        TRAJECTORY_5L,
        DROP_L,
        TRAJECTORY_6L,
        RETRACT_L,
        TRAJECTORY_7L,
        TRAJECTORY_8L,
        TRAJECTORY_1M,
        TRAJECTORY_2M,
        TURN_1M,
        TRAJECTORY_3M,
        TRAJECTORY_4M,
        DROP_M,
        TRAJECTORY_5M,
        RETRACT_M,
        TRAJECTORY_6M,
        TRAJECTORY_7M,
        TRAJECTORY_1R,
        TRAJECTORY_2R,
        TRAJECTORY_3R,
        TRAJECTORY_4R,
        TRAJECTORY_5R,
        TURN_1R,
        TRAJECTORY_6R,
        TRAJECTORY_7R,
        DROP_R,
        TRAJECTORY_8R,
        RETRACT_R,
        TRAJECTORY_9R,
        TRAJECTORY_10R,
        IDLE            // Our bot will enter the IDLE state when done
    }

    // We define the current state we're on
    // Default to IDLE
    State currentState = State.IDLE;

    // Define our start pose
    // This assumes we start at x: 15, y: 10, heading: 180 degrees
    Pose2d startPose = new Pose2d(-12, -63, Math.toRadians(90));

    @Override
    public void runOpMode() throws InterruptedException {

        int leftSpike = 1;
        int midSpike = 2;
        int rightSpike = 3;
        int propPosition = midSpike;
        // Initialize our lift
        Arm arm = new Arm(hardwareMap);
        Lift lift = new Lift(hardwareMap);
        claw = hardwareMap.get(Servo.class, "claw");
        claw.setPosition(CLOSED_CLAW);
        // Initialize SampleMecanumDrive
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        // Set inital pose
        drive.setPoseEstimate(startPose);

        // Let's define our trajectories
        Trajectory trajectory1L = drive.trajectoryBuilder(startPose)
                .strafeLeft(11)
                .build();
        Trajectory trajectory2L = drive.trajectoryBuilder(trajectory1L.end())
                .forward(37)
                .build();
        Trajectory trajectory3L = drive.trajectoryBuilder(trajectory2L.end())
                .back(16)
                .build();
        double turnAngle1L = Math.toRadians(90);
        Trajectory trajectory4L = drive.trajectoryBuilder(trajectory3L.end().plus(new Pose2d(0, 0, turnAngle1L)))
                .addTemporalMarker(0, () -> {
                    lift.setTarget(Lift.Presets.LOW_DROP);
                    arm.setPosition(Arm.Presets.OUTTAKE_POSITION);
                })
                .lineToConstantHeading(new Vector2d(-48, -42))
                .build();
        Trajectory trajectory5L = drive.trajectoryBuilder(trajectory4L.end())
                .forward(3)
                .build();
//drop
        Trajectory trajectory6L = drive.trajectoryBuilder(trajectory5L.end())
                .back(3)
                .build();
        Trajectory trajectory7L = drive.trajectoryBuilder(trajectory6L.end())
                .strafeRight(32)
                .build();
        Trajectory trajectory8L = drive.trajectoryBuilder(trajectory7L.end())
                .forward(10)
                .build();


        Trajectory trajectory1M = drive.trajectoryBuilder(startPose)
                .forward(36)
                .build();
        Trajectory trajectory2M = drive.trajectoryBuilder(trajectory1M.end())
                .back(8.5)
                .build();
        double turnAngle1M = Math.toRadians(90);
        Trajectory trajectory3M = drive.trajectoryBuilder(trajectory2M.end().plus(new Pose2d(0, 0, turnAngle1M)))
                .addTemporalMarker(0, () -> {
                    lift.setTarget(Lift.Presets.LOW_DROP);
                    arm.setPosition(Arm.Presets.OUTTAKE_POSITION);
                })
                .lineToConstantHeading(new Vector2d(-48, -35.5))
                .build();
        Trajectory trajectory4M = drive.trajectoryBuilder(trajectory3M.end())
                .forward(3)
                .build();
//drop
        Trajectory trajectory5M = drive.trajectoryBuilder(trajectory4M.end())
                .back(3)
                .build();
        Trajectory trajectory6M = drive.trajectoryBuilder(trajectory5M.end())
                .strafeRight(26)
                .build();
        Trajectory trajectory7M = drive.trajectoryBuilder(trajectory6M.end())
                .forward(10)
                .build();

        
        Trajectory trajectory1R = drive.trajectoryBuilder(startPose)
                .forward(31)
                .build();
        Trajectory trajectory2R = drive.trajectoryBuilder(trajectory1R.end())
                .strafeRight(11)
                .build();
        Trajectory trajectory3R = drive.trajectoryBuilder(trajectory2R.end())
                .back(7)
                .build();
        Trajectory trajectory4R = drive.trajectoryBuilder(trajectory3R.end())
                .strafeLeft(10)
                .build();
        Trajectory trajectory5R = drive.trajectoryBuilder(trajectory4R.end())
                .forward(10)
                .build();
        double turnAngle1R = Math.toRadians(90);
        Trajectory trajectory6R = drive.trajectoryBuilder(trajectory5R.end().plus(new Pose2d(0, 0, turnAngle1R)))
                .addTemporalMarker(0, () -> {
                    lift.setTarget(Lift.Presets.LOW_DROP);
                    arm.setPosition(Arm.Presets.OUTTAKE_POSITION);
                })
                .lineToConstantHeading(new Vector2d(-48, -29))
                .build();
        Trajectory trajectory7R = drive.trajectoryBuilder(trajectory6R.end())
                .forward(3)
                .build();
//drop
        Trajectory trajectory8R = drive.trajectoryBuilder(trajectory7R.end())
                .back(3)
                .build();
        Trajectory trajectory9R = drive.trajectoryBuilder(trajectory8R.end())
                .strafeRight(18)
                .build();
        Trajectory trajectory10R = drive.trajectoryBuilder(trajectory9R.end())
                .forward(10)
                .build();

        ElapsedTime timer = new ElapsedTime();

        waitForStart();

        if (isStopRequested()) return;

        // Set the current state to TRAJECTORY_1, our first step
        // Then have it follow that trajectory
        // Make sure you use the async version of the commands
        // Otherwise it will be blocking and pause the program here until the trajectory finishes
        if(propPosition == leftSpike) {
            currentState = State.TRAJECTORY_1L;
            drive.followTrajectoryAsync(trajectory1L);
        } else if (propPosition == midSpike) {
            currentState = State.TRAJECTORY_1M;
            drive.followTrajectoryAsync(trajectory1M);
        } else {
            currentState = State.TRAJECTORY_1R;
            drive.followTrajectoryAsync(trajectory1R);
        }

        while (opModeIsActive() && !isStopRequested()) {
            // Our state machine logic
            // You can have multiple switch statements running together for multiple state machines
            // in parallel. This is the basic idea for subsystems and commands.

            // We essentially define the flow of the state machine through this switch statement
            switch (currentState) {
                case TRAJECTORY_1L:
                    // Check if the drive class isn't busy
                    // `isBusy() == true` while it's following the trajectory
                    // Once `isBusy() == false`, the trajectory follower signals that it is finished
                    // We move on to the next state
                    // Make sure we use the async follow function
                    //lift.setTarget(2400);
                    if (!drive.isBusy()) {
                        currentState = State.TRAJECTORY_2L;
                        drive.followTrajectoryAsync(trajectory2L);
                    }
                    break;
                case TRAJECTORY_2L:
                    if (!drive.isBusy()) {
                        currentState = State.TRAJECTORY_3L;
                        drive.followTrajectoryAsync(trajectory3L);
                    }
                    break;
                case TRAJECTORY_3L:
                    if (!drive.isBusy()) {
                        currentState = State.TURN_1L;
                        drive.turnAsync(turnAngle1L);
                    }
                    break;
                case TURN_1L:
                    if (!drive.isBusy()) {
                        currentState = State.TRAJECTORY_4L;
                        drive.followTrajectoryAsync(trajectory4L);
                    }
                    break;
                case TRAJECTORY_4L:
                    if (!drive.isBusy()) {
                        currentState = State.TRAJECTORY_5L;
                        drive.followTrajectoryAsync(trajectory5L);
                    }
                    break;
                case TRAJECTORY_5L:
                    if (!drive.isBusy()) {
                        currentState = State.DROP_L;
                        timer.reset();
                        arm.setDropping(true);
                    }
                    break;
                case DROP_L:
                    if (timer.seconds() > 1.5) {
                        arm.setDropping(false);
                        currentState = State.TRAJECTORY_6L;
                        drive.followTrajectoryAsync(trajectory6L);
                    }
                    break;
                case TRAJECTORY_6L:
                    if (!drive.isBusy()) {
                        currentState = State.RETRACT_L;
                    }
                    break;
                case RETRACT_L:
                    timer.reset();
                    lift.setTarget(Lift.Presets.LOW_DROP);
                    arm.setPosition(Arm.Presets.OUTTAKE_POSITION);
                    if (timer.seconds() > 1.5) {
                        currentState = State.TRAJECTORY_7L;
                        drive.followTrajectoryAsync(trajectory7L);
                    }
                    break;
                case TRAJECTORY_7L:
                    if (!drive.isBusy()) {
                        currentState = State.TRAJECTORY_8L;
                        drive.followTrajectoryAsync(trajectory8L);
                    }
                    break;
                case TRAJECTORY_8L:
                    if (!drive.isBusy()) {
                        currentState = State.IDLE;
                    }
                    break;







                case TRAJECTORY_1M:
                    if (!drive.isBusy()) {
                        currentState = State.TRAJECTORY_2M;
                        drive.followTrajectoryAsync(trajectory2M);
                    }
                    break;
                case TRAJECTORY_2M:
                    if (!drive.isBusy()) {
                        currentState = State.TURN_1M;
                        drive.turnAsync(turnAngle1M);
                    }
                    break;
                case TURN_1M:
                    if (!drive.isBusy()) {
                        currentState = State.TRAJECTORY_3M;
                        drive.followTrajectoryAsync(trajectory3M);
                    }
                    break;
                case TRAJECTORY_3M:
                    if (!drive.isBusy()) {
                        currentState = State.TRAJECTORY_4M;
                        drive.followTrajectoryAsync(trajectory4M);
                    }
                    break;
                case TRAJECTORY_4M:
                    if (!drive.isBusy()) {
                        currentState = State.DROP_M;
                        timer.reset();
                        arm.setDropping(true);
                    }
                    break;
                case DROP_M:
                    if (timer.seconds() > 1.5) {
                        arm.setDropping(false);
                        currentState = State.TRAJECTORY_5M;
                        drive.followTrajectoryAsync(trajectory5M);
                    }
                    break;
                case TRAJECTORY_5M:
                    if (!drive.isBusy()) {
                        currentState = State.RETRACT_M;
                    }
                    break;
                case RETRACT_M:
                    timer.reset();
                    lift.setTarget(Lift.Presets.LOW_DROP);
                    arm.setPosition(Arm.Presets.OUTTAKE_POSITION);
                    if (timer.seconds() > 1.5) {
                        currentState = State.TRAJECTORY_6M;
                        drive.followTrajectoryAsync(trajectory6M);
                    }
                    break;
                case TRAJECTORY_6M:
                    if (!drive.isBusy()) {
                        currentState = State.TRAJECTORY_7M;
                        drive.followTrajectoryAsync(trajectory7M);
                    }
                    break;
                case TRAJECTORY_7M:
                    if (!drive.isBusy()) {
                        currentState = State.IDLE;
                    }
                    break;




                case TRAJECTORY_1R:
                    if (!drive.isBusy()) {
                        currentState = State.TRAJECTORY_2R;
                        drive.followTrajectoryAsync(trajectory2R);
                    }
                    break;
                case TRAJECTORY_2R:
                    if (!drive.isBusy()) {
                        currentState = State.TRAJECTORY_3R;
                        drive.followTrajectoryAsync(trajectory3R);
                    }
                    break;
                case TRAJECTORY_3R:
                    if (!drive.isBusy()) {
                        currentState = State.TRAJECTORY_4R;
                        drive.followTrajectoryAsync(trajectory4R);
                    }
                    break;
                case TRAJECTORY_4R:
                    if (!drive.isBusy()) {
                        currentState = State.TRAJECTORY_5R;
                        drive.followTrajectoryAsync(trajectory5R);
                    }
                    break;
                case TRAJECTORY_5R:
                    if (!drive.isBusy()) {
                        currentState = State.TURN_1R;
                        drive.turnAsync(turnAngle1R);
                    }
                    break;
                case TURN_1R:
                    if (!drive.isBusy()) {
                        currentState = State.TRAJECTORY_6R;
                        drive.followTrajectoryAsync(trajectory6R);
                    }
                    break;
                case TRAJECTORY_6R:
                    if (!drive.isBusy()) {
                        currentState = State.TRAJECTORY_7R;
                        drive.followTrajectoryAsync(trajectory7R);
                    }
                    break;
                case TRAJECTORY_7R:
                    if (!drive.isBusy()) {
                        currentState = State.DROP_R;
                        timer.reset();
                        arm.setDropping(true);
                    }
                    break;
                case DROP_R:
                    if (timer.seconds() > 1.5) {
                        arm.setDropping(false);
                        currentState = State.TRAJECTORY_8R;
                        drive.followTrajectoryAsync(trajectory8R);
                    }
                    break;
                case TRAJECTORY_8R:
                    if (!drive.isBusy()) {
                        currentState = State.RETRACT_R;
                    }
                    break;
                case RETRACT_R:
                    timer.reset();
                    lift.setTarget(Lift.Presets.LOW_DROP);
                    arm.setPosition(Arm.Presets.OUTTAKE_POSITION);
                    if (timer.seconds() > 1.5) {
                        currentState = State.TRAJECTORY_9R;
                        drive.followTrajectoryAsync(trajectory9R);
                    }
                    break;
                case TRAJECTORY_9R:
                    if (!drive.isBusy()) {
                        currentState = State.TRAJECTORY_10R;
                        drive.followTrajectoryAsync(trajectory10R);
                    }
                    break;
                case TRAJECTORY_10R:
                    if (!drive.isBusy()) {
                        currentState = State.IDLE;
                    }
                    break;


                case IDLE:
                    // Do nothing in IDLE
                    // currentState does not change once in IDLE
                    // This concludes the autonomous program
                    break;
            }

            // Anything outside of the switch statement will run independent of the currentState

            // We update drive continuously in the background, regardless of state
            drive.update();
            lift.update();
            arm.update();
            // Read pose
            Pose2d poseEstimate = drive.getPoseEstimate();

            // Continually write pose to `PoseStorage`
            //PoseStorage.currentPose = poseEstimate;

            // Print pose to telemetry
            telemetry.addData("x", poseEstimate.getX());
            telemetry.addData("y", poseEstimate.getY());
            telemetry.addData("heading", poseEstimate.getHeading());
            telemetry.addData("slide target", lift.getTarget());
            telemetry.addData("state ", currentState);
            telemetry.update();
        }
    }
}