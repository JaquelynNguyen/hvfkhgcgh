package org.firstinspires.ftc.teamcode.opmodes;

//import com.acmerobotics.roadrunner.geometry.Pose2d;

//import com.acmerobotics.roadrunner.trajectory.Trajectory;

//import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;


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
/*
@Autonomous(name = "CloseBlue", group = "advanced")
@Disabled
public class CloseBlue extends LinearOpMode {
    private Servo wrist;
    private Servo leftClaw;
    private Servo rightClaw;

    private Servo arm_left;
    private Servo arm_right;

    final double OPEN_CLAW = .46;
    final double CLOSED_CLAW = 0;

    public enum V4bState {
        V4B_START,
        V4B_MID,
        V4B_WRISTVERT,
        V4B_MOVE, //sadfasdf
        V4B_DROPWAIT,
        V4B_MOVE2
    };
    V4bState v4bState = V4bState.V4B_START;

    // This enum defines our "state"
    // This is essentially just defines the possible steps our program will take
    enum State {

        TRAJECTORY_1,   //drive 30 to in between spikes
        WAIT_0,         //wait in between
        TRAJECTORY_2,   //drive back 4 to placing position
        WAIT_1,         //purple placing
        TURN_1L,        //turn to left spike
        TURN_1R,        //turn to right spike
        TRAJECTORY_3,   //move back 6 if right spike
        TURN_2,         //turn to face backdrop
        TRAJECTORY_4M,  //drive to backdrop (mid)
        TRAJECTORY_4R,  //drive to backdrop (right)
        TRAJECTORY_5,   //strafing 24 to wall (left)
        TRAJECTORY_6,   //drive to backdrop level (left)
        TRAJECTORY_7L,  //strafe right to left apriltag
        TRAJECTORY_7R,  //strafe right to right apriltag
        WAIT_2,         //place yellow
     
        IDLE            // Our bot will enter the IDLE state when done
    }


    // We define the current state we're on
    // Default to IDLE
    State currentState = State.IDLE;

    boolean purpDropStart = false;
    boolean purpDropEnd = false;
    boolean yellowDropStart = false;
    boolean yellowDropEnd = false;
    int leftSpike = 1;
    int midSpike = 2;
    int rightSpike = 3;
    int propPosition = leftSpike;

    final double ARM_MID_FRONT = 0.1;
    final double ARM_MID_BACK = 0.9;
    final double HORZ_WRIST = 0;
    final double VERT_WRIST = 1;

    final double ARM_PURP_DROP = 1;
    final double ARM_BACKDROP_DROP = 0.8;
    // Define our start pose
    // This assumes we start at x: 15, y: 10, heading: 180 degrees
    Pose2d startPose = new Pose2d(-62, 12, Math.toRadians(0));

    @Override
    public void runOpMode() throws InterruptedException {
        // Initialize our lift
        wrist = hardwareMap.get(Servo.class, "wrist");
        leftClaw = hardwareMap.get(Servo.class, "leftClaw");
        rightClaw = hardwareMap.get(Servo.class, "rightClaw");
        arm_left = hardwareMap.get(Servo.class, "arm_left");
        arm_right = hardwareMap.get(Servo.class, "arm_right");
        Drivetrain lift = new Drivetrain(hardwareMap);
        leftClaw.setPosition(CLOSED_CLAW);
        rightClaw.setPosition(CLOSED_CLAW);
        // Initialize SampleMecanumDrive
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        // Set inital pose
        drive.setPoseEstimate(startPose);


        Trajectory trajectory1 = drive.trajectoryBuilder(startPose)
                .forward(30)
                .build();

        Trajectory trajectory2 = drive.trajectoryBuilder(trajectory1.end())
                .back(4)
                .build();

        Trajectory trajectory3 = drive.trajectoryBuilder(trajectory2.end().plus(new Pose2d(0, 0, Math.toRadians(-90))))
                .back(6)
                .build();

        Trajectory trajectory4M = drive.trajectoryBuilder(trajectory2.end().plus(new Pose2d(0, 0, Math.toRadians(90))))
                .forward(36)
                .build();
        Trajectory trajectory4R = drive.trajectoryBuilder(trajectory3.end().plus(new Pose2d(0, 0, Math.toRadians(180))))
                .forward(30)
                .build();

        Trajectory trajectory5 = drive.trajectoryBuilder(trajectory2.end().plus(new Pose2d(0, 0, Math.toRadians(90))))
                .strafeRight(22)
                .build();

        Trajectory trajectory6 = drive.trajectoryBuilder(trajectory5.end())
                .forward(36)
                .build();

        Trajectory trajectory7L = drive.trajectoryBuilder(trajectory6.end())
                .strafeLeft(28)
                .build();
        Trajectory trajectory7R = drive.trajectoryBuilder(trajectory4R.end())
                .strafeRight(6)
                .build();




        ElapsedTime v4bTimer = new ElapsedTime();
        ElapsedTime timer0 = new ElapsedTime();

        waitForStart();

        if (isStopRequested()) return;

        // Set the current state to TRAJECTORY_1, our first step
        // Then have it follow that trajectory
        // Make sure you use the async version of the commands
        // Otherwise it will be blocking and pause the program here until the trajectory finishes
        currentState = State.TRAJECTORY_1;
        drive.followTrajectoryAsync(trajectory1);

        while (opModeIsActive() && !isStopRequested()) {
            // Our state machine logic
            // You can have multiple switch statements running together for multiple state machines
            // in parallel. This is the basic idea for subsystems and commands.

            // We essentially define the flow of the state machine through this switch statement
            switch (currentState) {
                case TRAJECTORY_1:
                    // Check if the drive class isn't busy
                    // `isBusy() == true` while it's following the trajectory
                    // Once `isBusy() == false`, the trajectory follower signals that it is finished
                    // We move on to the next state
                    // Make sure we use the async follow function
                    //lift.setTarget(2400);
                    if (!drive.isBusy()) {
                        timer0.reset();
                        currentState = State.WAIT_0;
                    }
                    break;
                case WAIT_0:
                    if(timer0.seconds() > 0.4) {
                        currentState = State.TRAJECTORY_2;
                        drive.followTrajectoryAsync(trajectory2);
                    }
                    break;
                case TRAJECTORY_2:
                    if(!drive.isBusy()) {
                        if(propPosition == leftSpike) {
                            currentState = State.TURN_1L;
                            drive.turnAsync(Math.toRadians(90));
                        } else if (propPosition == rightSpike) {
                            currentState = State.TURN_1R;
                            drive.turnAsync(Math.toRadians(-90));
                        } else {
                            currentState = State.WAIT_1;
                        }
                    }
                case TURN_1L:
                    // Check if the drive class is busy turning
                    // If not, move onto the next state, TRAJECTORY_3, once finished
                    if (!drive.isBusy()) {
                        currentState = State.WAIT_1;
                    }
                    break;
                case TURN_1R:
                    // Check if the drive class is busy turning
                    // If not, move onto the next state, TRAJECTORY_3, once finished
                    if (!drive.isBusy()) {
                        currentState = State.WAIT_1;
                    }
                    break;
                case WAIT_1:
                    //todo: PURPLE DROP CODE
                    purpDropStart = true;
                    //v4b code
                    switch (v4bState) {
                        case V4B_START:
                            // Waiting for some input
                            if (purpDropStart) {
                                arm_left.setPosition(ARM_MID_BACK);
                                arm_right.setPosition(ARM_MID_BACK);
                                v4bTimer.reset();
                                v4bState = V4bState.V4B_MID;
                            }
                            break;
                        case V4B_MID:
                            if (v4bTimer.seconds() > 0.15) {
                                wrist.setPosition(VERT_WRIST);
                                v4bTimer.reset();
                                v4bState = V4bState.V4B_WRISTVERT;
                            }
                        case V4B_WRISTVERT:
                            // if wrist finished moving, then move on
                            // otherwise do nothing
                            if (v4bTimer.seconds() > 0.3) {
                                arm_left.setPosition(ARM_PURP_DROP);
                                arm_right.setPosition(ARM_PURP_DROP);
                                v4bTimer.reset();
                                v4bState = V4bState.V4B_MOVE;
                            }
                            break;
                        case V4B_MOVE:
                            // if arm finished moving, then move on
                            // otherwise do nothing
                            if (v4bTimer.seconds() > 0.5) {
                                leftClaw.setPosition(OPEN_CLAW);
                                v4bTimer.reset();
                                v4bState = V4bState.V4B_DROPWAIT;
                            }
                            break;
                        case V4B_DROPWAIT:
                            // if wrist finished moving, then back to start
                            // otherwise do nothing
                            if (v4bTimer.seconds() > 0.3) {
                                v4bState = V4bState.V4B_MOVE2;
                                arm_left.setPosition(ARM_MID_BACK);
                                arm_right.setPosition(ARM_MID_BACK);
                                v4bTimer.reset();
                            }
                            break;
                        case V4B_MOVE2:
                            if (v4bTimer.seconds() > 0.3) {
                                v4bState = V4bState.V4B_START;
                                purpDropEnd = true;
                            }
                            break;
                    }
                    if(purpDropEnd) {
                        if (propPosition == rightSpike) {
                            currentState = State.TRAJECTORY_3;
                            drive.followTrajectoryAsync(trajectory3);
                        } else if (propPosition == midSpike) {
                            currentState = State.TURN_2;
                            drive.turnAsync(Math.toRadians(90));
                        } else {
                            currentState = State.TRAJECTORY_5;
                            drive.followTrajectoryAsync(trajectory5);
                        }
                    }
                    break;
                case TRAJECTORY_3:
                    // Check if the drive class is busy following the trajectory
                    // Move on to the next state, TURN_1, once finished
                    if (!drive.isBusy()) {
                        currentState = State.TURN_2;
                        drive.turnAsync(Math.toRadians(180));
                    }
                    break;
                case TURN_2:
                    // Check if the drive class is busy turning
                    // If not, move onto the next state, TRAJECTORY_3, once finished
                    if (!drive.isBusy()) {
                        if(propPosition == midSpike) {
                            currentState = State.TRAJECTORY_4M;
                            drive.followTrajectoryAsync(trajectory4M);
                        } else {
                            currentState = State.TRAJECTORY_4R;
                            drive.followTrajectoryAsync(trajectory4R);
                        }
                    }
                    break;
                case TRAJECTORY_4M:
                    if (!drive.isBusy()) {
                        currentState = State.WAIT_2;
                    }
                    break;
                case TRAJECTORY_4R:
                    if (!drive.isBusy()) {
                        currentState = State.TRAJECTORY_7R;
                        drive.followTrajectoryAsync(trajectory7R);
                    }
                    break;
                case TRAJECTORY_7R:
                    if (!drive.isBusy()) {
                        currentState = State.WAIT_2;
                    }
                    break;
                case TRAJECTORY_5:
                    if (!drive.isBusy()) {
                        currentState = State.TRAJECTORY_6;
                        drive.followTrajectoryAsync(trajectory6);
                    }
                    break;
                case TRAJECTORY_6:
                    if (!drive.isBusy()) {
                        currentState = State.TRAJECTORY_7L;
                        drive.followTrajectoryAsync(trajectory7L);
                    }
                    break;
                case TRAJECTORY_7L:
                    if (!drive.isBusy()) {
                        currentState = State.WAIT_2;
                    }
                    break;
                case WAIT_2:
                    //todo: YELLOW DROP CODE
                    yellowDropStart = true;
                    //v4b code
                    switch (v4bState) {
                        case V4B_START:
                            // Waiting for some input
                            if (yellowDropStart) {
                                v4bTimer.reset();
                                arm_left.setPosition(ARM_BACKDROP_DROP);
                                arm_right.setPosition(ARM_BACKDROP_DROP);
                                v4bState = V4bState.V4B_MOVE;
                            }
                            break;
                        case V4B_MOVE:
                            // if arm finished moving, then move on
                            // otherwise do nothing
                            if (v4bTimer.seconds() > 0.5) {
                                leftClaw.setPosition(OPEN_CLAW);
                                v4bTimer.reset();
                                v4bState = V4bState.V4B_DROPWAIT;
                            }
                            break;
                        case V4B_DROPWAIT:
                            // if wrist finished moving, then back to start
                            // otherwise do nothing
                            if (v4bTimer.seconds() > 0.3) {
                                v4bState = V4bState.V4B_MOVE2;
                                arm_left.setPosition(ARM_MID_BACK);
                                arm_right.setPosition(ARM_MID_FRONT);
                                v4bTimer.reset();
                            }
                            break;
                        case V4B_MOVE2:
                            if (v4bTimer.seconds() > 0.3) {
                                v4bState = V4bState.V4B_START;
                                purpDropEnd = true;
                            }
                            break;
                    }
                    currentState = State.IDLE;
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
            //arm.update();
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

 */