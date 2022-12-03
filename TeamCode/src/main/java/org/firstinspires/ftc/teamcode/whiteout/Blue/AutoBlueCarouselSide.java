package org.firstinspires.ftc.teamcode.whiteout.Blue;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.constraints.AngularVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.MecanumVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.MinVelocityConstraint;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.teamcode.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.whiteout.Common.PoseStorage;
import org.firstinspires.ftc.teamcode.whiteout.Common.tataAutonomousBase;
import org.firstinspires.ftc.teamcode.whiteout.RobotSideArm.RobotSideArmDriver;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

import java.util.Arrays;

@Autonomous(name = "Blue - Auto - Carousel Side", group = "BLUE")
public class AutoBlueCarouselSide extends tataAutonomousBase {

    public Pose2d startPose = new Pose2d( -40.75, 63.5, Math.toRadians( 90 ));

    //Optimal pos for dropping in Levels 1, 2 or 3
    public Pose2d dropPose[] = {new Pose2d(),                            //Not Used
            new Pose2d( -29.2, 33.75, Math.toRadians( 180-30 )),  //level 1
            //new Pose2d( -30, 34.25, Math.toRadians( 180-30 )),  //level 1
            //new Pose2d( -30.92, 32.75, Math.toRadians( 180-30 )),  //level 1
            //new Pose2d( -31.5, 34.25, Math.toRadians( 180-30 )),  //level 2
            //new Pose2d( -30.9, 34.75, Math.toRadians( 180-30 )),  //level 2
            new Pose2d( -30.06, 34.25, Math.toRadians( 180-30 )),  //level 2
            new Pose2d( -35.85,36.75, Math.toRadians( 180-30 ))}; //level 3



    @Override
    public void runOpMode() throws InterruptedException {

        init(hardwareMap, startPose, opModeCalled.AUTO);
        robot.setPoseEstimate(startPose);

        while (!isStopRequested() && !isStarted()) {
           barCodeLoc = getMarkerPos();
           telemetry.addData("Waiting to Start. Element position", barCodeLoc);
           telemetry.update();
        }

        waitForStart();

        telemetry.addData("Started. Element position", barCodeLoc);
        telemetry.update();

        //Dont need Main camera Anymore
        //stopMainCamera();

        if (isStopRequested()) {
            stopThreads();
            return;
        }

        TrajectorySequence moveToDropGE = getTrajectorySequenceBuilder()
                .setVelConstraint( new MinVelocityConstraint( Arrays.asList(new AngularVelocityConstraint(DriveConstants.MAX_ANG_VEL ), new MecanumVelocityConstraint( 40, DriveConstants.TRACK_WIDTH ) ) ) )
                .addTemporalMarker(() -> {
                    moveSlideToPosShank(barCodeLoc, SlideDirection.OUT);
                })
                .lineToSplineHeading(dropPose[barCodeLoc])
                .waitSeconds(0.2)
                .addTemporalMarker(() -> {
                    slideDriver.dropGameElement();
                })
                .waitSeconds(0.1)

                .addTemporalMarker(() -> {
                    moveSlideToPosShank(barCodeLoc, SlideDirection.IN);
                    //sideArmDriver.activateSideArms(RobotSideArmDriver.RobotSideArmSide.LEFT_SIDE, RobotSideArmDriver.RobotSideArmPreSetPos.DOWN);
                })
                .setTangent(Math.toRadians(90))
                .splineToLinearHeading(new Pose2d( -55, 48, Math.toRadians( 90)), Math.toRadians(200))
                .build();
        robot.followTrajectorySequence(moveToDropGE);
        PoseStorage.currentPose = robot.getPoseEstimate();

        //Correct Robot Orientation
        imuParams = imuDriver.getRobotImuParams();
        robot.turn(-1 * Math.toRadians(imuParams.correctedHeading - (90 + 3)));

        dsParams = sensorDriver.getRobotSensorParams();

        RobotLog.ii("SHANK", "Duck Side Red - RS %.2f, F %.2f", dsParams.x_LS, dsParams.x_RF);


        TrajectorySequence moveToStartCarousel = getTrajectorySequenceBuilder()
                .setVelConstraint( new MinVelocityConstraint( Arrays.asList(new AngularVelocityConstraint(DriveConstants.MAX_ANG_VEL ), new MecanumVelocityConstraint( 15, DriveConstants.TRACK_WIDTH ) ) ) )
                .strafeLeft(Math.min(dsParams.x_LS - 1, 10))
                .waitSeconds(0.2)
                .forward(Math.min(dsParams.x_RF - 7, 20))  //Carousel if of radius 7.5 inch //was using LF
                .addTemporalMarker(() -> {
                    //start Carosel motor
                    sideArmDriver.activateSideArms(RobotSideArmDriver.RobotSideArmSide.RIGHT_SIDE, RobotSideArmDriver.RobotSideArmPreSetPos.DOWN);
                    crDriver.toggleCarousel(true);
                })
                .waitSeconds(3)
                .addTemporalMarker(() -> {
                    //stop Carosel motor
                    crDriver.toggleCarousel(false);
                    sideArmDriver.activateSideArms(RobotSideArmDriver.RobotSideArmSide.RIGHT_SIDE, RobotSideArmDriver.RobotSideArmPreSetPos.UP);

                })
                .back(1)
                .build();

        robot.followTrajectorySequence(moveToStartCarousel);
        PoseStorage.currentPose = robot.getPoseEstimate();

        //Correct Robot Orientation
        imuParams = imuDriver.getRobotImuParams();
        robot.turn(-1 * Math.toRadians(imuParams.correctedHeading - (90)));

        TrajectorySequence moveToStartCarousel1 = getTrajectorySequenceBuilder()
                .setVelConstraint( new MinVelocityConstraint( Arrays.asList(new AngularVelocityConstraint(DriveConstants.MAX_ANG_VEL ), new MecanumVelocityConstraint( 15, DriveConstants.TRACK_WIDTH ) ) ) )
                .strafeRight(4)
                .addTemporalMarker(() -> {
                    sideArmDriver.activateSideArms(RobotSideArmDriver.RobotSideArmSide.LEFT_SIDE, RobotSideArmDriver.RobotSideArmPreSetPos.DOWN);
                })
                .waitSeconds(0.5)
                .forward(3)
                .strafeRight(18)
                .back(4)
                .addTemporalMarker(() -> {
                    sideArmDriver.activateSideArms(RobotSideArmDriver.RobotSideArmSide.BOTH_SIDES, RobotSideArmDriver.RobotSideArmPreSetPos.UP);
                })
                //.waitSeconds(1)
                .strafeLeft(8)
                .addTemporalMarker(() -> {
                    inTakeDriver.intakeSet(true, true);
                })
                .build();

        robot.followTrajectorySequence(moveToStartCarousel1);
        PoseStorage.currentPose = robot.getPoseEstimate();

        //Correct Robot Orientation
        imuParams = imuDriver.getRobotImuParams();
        robot.turn(-1 * Math.toRadians(imuParams.correctedHeading - (90)));

        //try collecting duck
        dsParams = sensorDriver.getRobotSensorParams();
        RobotLog.ii("SHANK", "Collecting  Duck Side Blue - RS %.2f, F %.2f", dsParams.x_LS, dsParams.x_RF);

        if (dsParams.x_RF > 2) {
            TrajectorySequence collectDuck = getTrajectorySequenceBuilder()
                    .forward(Math.min(dsParams.x_RF - 2, 15))
                    .build();
            robot.followTrajectorySequence(collectDuck);
            PoseStorage.currentPose = robot.getPoseEstimate();
        }

        boolean dropDuck = false;

        ElapsedTime stopTimer = new ElapsedTime();
        while(opModeIsActive() && !isStopRequested() && (stopTimer.seconds() < 3))
        {
            if (inTakeDriver.isElementCollected()) {
                //Game Elemented is collected
                dropDuck = true;
                break;
            }
            //Check if duck has been collected
            TrajectorySequence collectDuck = getTrajectorySequenceBuilder()
                    .strafeRight(2)
                    .build();
            robot.followTrajectorySequence(collectDuck);
            PoseStorage.currentPose = robot.getPoseEstimate();
        }

        if (dropDuck == true) {
            //Drop Duck to level
            int duck_drop_level = 1;
            TrajectorySequence moveToDropDuck = getTrajectorySequenceBuilder()
                    .waitSeconds(0.2)
                    .addTemporalMarker(() -> {
                        inTakeDriver.intakeSet(false, true);
                    })
                    .addTemporalMarker(() -> {
                        moveSlideToPosShank(duck_drop_level, SlideDirection.OUT);
                    })
                    .lineToSplineHeading(new Pose2d(-29.5, 33.75, Math.toRadians( 180-30))) // was Pose2d[duck_drop_level]
                    .waitSeconds(0.2)
                    .addTemporalMarker(() -> {
                        slideDriver.dropGameElement();
                    })
                    .waitSeconds(0.1)

                    .addTemporalMarker(() -> {
                        moveSlideToPosShank(duck_drop_level, SlideDirection.IN);
                    })
                    .lineToLinearHeading(new Pose2d(-65, 38, Math.toRadians(180)))
                    .strafeLeft(2)
                    .build();
            robot.followTrajectorySequence(moveToDropDuck);
        } else {
            //No duck directly park
            TrajectorySequence park = getTrajectorySequenceBuilder()
                    .addTemporalMarker(() -> {
                        inTakeDriver.intakeSet(false, true);
                    })
                    .lineToLinearHeading(new Pose2d(-65, 38, Math.toRadians(180)))
                    .strafeLeft(2)
                    .build();
            robot.followTrajectorySequence(park);
        }
        dsParams = sensorDriver.getRobotSensorParams();

        RobotLog.ii("SHANK", "After Park Duck Side Blue - LS %.2f, F %.2f", dsParams.x_LS, dsParams.x_RF);
        double distance_from_front_wall = 25.5;
        if (dsParams.x_RF < distance_from_front_wall) {
            //Check if duck has been collected
            TrajectorySequence park = getTrajectorySequenceBuilder()
                    .waitSeconds(0.2)
                    .back(Math.min(distance_from_front_wall - dsParams.x_RF, 3))
                    .build();
            robot.followTrajectorySequence(park);
            PoseStorage.currentPose = robot.getPoseEstimate();
        }

        stopThreads();

    }
}
