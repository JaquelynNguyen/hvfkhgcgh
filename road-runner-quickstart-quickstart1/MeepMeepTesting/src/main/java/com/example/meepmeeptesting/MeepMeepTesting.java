package com.example.meepmeeptesting;


import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

import java.awt.Image;
import java.io.File;
import java.io.IOException;

import javax.imageio.ImageIO;

public class MeepMeepTesting {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(800);

        RoadRunnerBotEntity redBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)
                .setDimensions(16.5, 14)
                .followTrajectorySequence(drive ->
                                drive.trajectorySequenceBuilder(new Pose2d(12, -63, Math.toRadians(90)))
                                        //left

                                        .forward(31)
                                        .strafeLeft(11)
                                        .back(7)
                                        .waitSeconds(1)
                                        .strafeRight(10)
                                        .forward(10)
                                        .turn(Math.toRadians(-90))
                                        .lineToConstantHeading(new Vector2d(48, -29))
                                        .forward(3)
                                        .waitSeconds(1)
                                        .back(3)
                                        .strafeLeft(18)
                                        .forward(10)


                                        //middle
/*
                                        .forward(36)
                                        .back(8.5)
                                        .waitSeconds(1)
                                        .turn(Math.toRadians(-90))
                                        .lineToConstantHeading(new Vector2d(48, -35.5))
                                        .forward(3)
                                        .waitSeconds(1)
                                        .back(3)
                                        .strafeLeft(26)
                                        .forward(10)

 */


                                        //right
/*
                                        .strafeRight(11)
                                        .forward(37)
                                        .back(16)
                                        .turn(Math.toRadians(-90))
                                        .waitSeconds(1)
                                        .lineToConstantHeading(new Vector2d(48, -42))
                                        .forward(3)
                                        .waitSeconds(1)
                                        .back(3)
                                        .strafeLeft(32)
                                        .forward(10)

 */

                                        .build()


                );


        RoadRunnerBotEntity blueBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)
                .setDimensions(16.5, 14)
                .followTrajectorySequence(drive ->
                                drive.trajectorySequenceBuilder(new Pose2d(-12, -63, Math.toRadians(90)))
                                        //left

                                        .strafeLeft(11)
                                        .forward(37)
                                        .back(16)
                                        .turn(Math.toRadians(90))
                                        .waitSeconds(1)
                                        .lineToConstantHeading(new Vector2d(-48, -42))
                                        .forward(3)
                                        .waitSeconds(1)
                                        .back(3)
                                        .strafeRight(32)
                                        .forward(10)






                                        //middle
/*
                                        .forward(36)
                                        .back(8.5)
                                        .waitSeconds(1)
                                        .turn(Math.toRadians(90))
                                        .lineToConstantHeading(new Vector2d(-48, -35.5))
                                        .forward(3)
                                        .waitSeconds(1)
                                        .back(3)
                                        .strafeRight(26)
                                        .forward(10)

 */





                                        //right
/*
                                        .forward(31)
                                        .strafeRight(11)
                                        .back(7)
                                        .waitSeconds(1)
                                        .strafeLeft(10)
                                        .forward(10)
                                        .turn(Math.toRadians(90))
                                        .lineToConstantHeading(new Vector2d(-48, -29))
                                        .forward(3)
                                        .waitSeconds(1)
                                        .back(3)
                                        .strafeRight(18)
                                        .forward(10)

 */

                                        .build()
                );

        Image img = null;
        try { img = ImageIO.read(new File("C:\\Users\\Jackie\\Documents\\field.png")); }
        catch (IOException e) {}

        meepMeep.setBackground(MeepMeep.Background.FIELD_CENTERSTAGE_JUICE_DARK)
            //    meepMeep.setBackground(img)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(blueBot)
                .start();
    }
}

