package org.firstinspires.ftc.teamcode.whiteout.Blue;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.teamcode.whiteout.Common.PoseStorage;
import org.firstinspires.ftc.teamcode.whiteout.Common.tataAutonomousBase;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

@Autonomous(name = "Blue - Auto - Warehouse - Manav", group = "BLUE")
public class AutoBlueWarehouseManav extends tataAutonomousBase {

    public Pose2d startPose = new Pose2d(6.75, 60.5, Math.toRadians(90));
    private boolean isCollectLast = false;

    //Optimal pos for dropping in Levels 1, 2 or 3
    public Pose2d dropPose[] = {
            new Pose2d(),                            //Not Used
            new Pose2d(-2.3, 46.8, Math.toRadians(67.5)),//level 1
            new Pose2d(-2.2, 43.5, Math.toRadians(67.5)),//level 2
            new Pose2d(-1.9, 46.8, Math.toRadians(67.5)),
            new Pose2d(-3, 47, Math.toRadians(75)),
            new Pose2d(-3, 47, Math.toRadians(85))}; //level 3

    private double wallPos = 63;

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
        //barCodeLoc = 3;
        ElapsedTime stopTimer0 = new ElapsedTime();
        TrajectorySequence moveToDropGE = getTrajectorySequenceBuilder()

                .addTemporalMarker(() -> {
                    moveSlideToPos(barCodeLoc, SlideDirection.OUT);
                })
                .lineToSplineHeading(dropPose[barCodeLoc])
                .waitSeconds(0.15)
                .addTemporalMarker(() -> {
                    slideDriver.dropGameElement();
                })
                .waitSeconds(0.15)

                .addTemporalMarker(() -> {
                    moveSlideToPos(barCodeLoc, SlideDirection.IN);
                    //sideArmDriver.activateSideArms(RobotSideArmDriver.RobotSideArmPreSetPos.DOWN);
                })
                .setTangent(Math.toRadians(75))
                //.splineToSplineHeading(new Pose2d(14, wallPos, Math.toRadians(0)), Math.toRadians(0))
                .lineToLinearHeading( new Pose2d( 4, wallPos , Math.toRadians( 0 ) ))
                .build();
        robot.followTrajectorySequence(moveToDropGE);
        PoseStorage.currentPose = robot.getPoseEstimate();
        dsParams = sensorDriver.getRobotSensorParams();
        TrajectorySequence moveToDropGE2 = getTrajectorySequenceBuilder()
                //.strafeLeft(2.5)
                .strafeLeft(dsParams.x_LS)
                .forward(38)
                //.lineToConstantHeading(new Vector2d(42, 63))
                .addTemporalMarker(() -> {
                    inTakeDriver.intakeSet(true, true);
                })
                .build();
        robot.followTrajectorySequence(moveToDropGE2);
        PoseStorage.currentPose = robot.getPoseEstimate();



        //Correct Robot Orientation
        //imuParams = imuDriver.getRobotImuParams();
        //robot.turn(-1 * Math.toRadians(imuParams.correctedHeading - 270));



        RobotLog.ii("SHANK", "Duck Side Red - RS %.2f, F %.2f", dsParams.x_RS, dsParams.x_LF);


        //Check if duck has been collected

        ElapsedTime stopTimer = new ElapsedTime();
        while (opModeIsActive() && !isStopRequested() && (stopTimer.seconds() < 5)) {
            if (inTakeDriver.isElementCollected()) {
                //Game Elemented is collected
                break;
            }

            TrajectorySequence collectDuck = getTrajectorySequenceBuilder()
                    .forward(3)
                    .build();

            robot.followTrajectorySequence(collectDuck);
            PoseStorage.currentPose = robot.getPoseEstimate();
        }
        //Drop Duck
        dsParams = sensorDriver.getRobotSensorParams();
        TrajectorySequence moveToDropDuck = getTrajectorySequenceBuilder()
                .addTemporalMarker(() -> {
                    inTakeDriver.intakeSet(true, false);
                })
                .waitSeconds(0.5)
                //.back(10)
                //.strafeLeft(3)
                .setTangent(Math.toRadians(180))

                .addTemporalMarker(() -> {
                    moveSlideToPos(3, SlideDirection.OUT);
                })

                .lineToConstantHeading(new Vector2d(4, 63 + Math.min(dsParams.x_LS, 3)))
                //prev 14

                .lineToLinearHeading(dropPose[4])
                .addTemporalMarker(() -> {
                    inTakeDriver.intakeSet(false, false);
                })
                //.splineToLinearHeading(dropPose[4], Math.toRadians(180 + 67.5))

                .waitSeconds(0.2)
                .addTemporalMarker(() -> {
                    slideDriver.dropGameElement();
                })

                .waitSeconds(0.2)
                .addTemporalMarker(() -> {
                    moveSlideToPos(3, SlideDirection.IN);
                    //sideArmDriver.activateSideArms(RobotSideArmDriver.RobotSideArmPreSetPos.DOWN);
                })
                .lineToLinearHeading( new Pose2d( 4, wallPos , Math.toRadians( 0 ) ))
                .build();
        robot.followTrajectorySequence(moveToDropDuck);
        PoseStorage.currentPose = robot.getPoseEstimate();
        dsParams = sensorDriver.getRobotSensorParams();

        TrajectorySequence inbetween = getTrajectorySequenceBuilder()
                .setTangent(Math.toRadians(75))
                //.splineToSplineHeading(new Pose2d(14, wallPos, Math.toRadians(0)), Math.toRadians(0))

                //.strafeLeft(3)
                .strafeLeft(dsParams.x_LS)
                //.lineToConstantHeading(new Vector2d(42, 63))
                //.forward(38)
                //.strafeRight(20)

                .forward(38)
                .addTemporalMarker(() -> {
                    inTakeDriver.intakeSet(true, true);
                })
                .build();
        robot.followTrajectorySequence(inbetween);


        ElapsedTime stopTimer1 = new ElapsedTime();
        while (opModeIsActive() && !isStopRequested() && (stopTimer1.seconds() < 5)) {
            if (inTakeDriver.isElementCollected()) {
                //Game Elemented is collected
                break;
            }

            TrajectorySequence collectDuck = getTrajectorySequenceBuilder()
                    .forward(3)
                    .build();

            robot.followTrajectorySequence(collectDuck);
            PoseStorage.currentPose = robot.getPoseEstimate();
        }

        dsParams = sensorDriver.getRobotSensorParams();
        TrajectorySequence moveToDropDuck1 = getTrajectorySequenceBuilder()

                .waitSeconds(0.4)
                .addTemporalMarker(() -> {
                    inTakeDriver.intakeSet(true, false);
                })
                .waitSeconds(0.5)
                .back(10)
                .strafeRight(22)
                /*
                .addTemporalMarker(() -> {
                    moveSlideToPos(3, SlideDirection.OUT);
                })

                .waitSeconds(0.6)
                .addTemporalMarker(() -> {
                    inTakeDriver.intakeSet(false, false);
                })
                //.setTangent(Math.toRadians(180))
                .back(10)
                .strafeRight(22)

                //.strafeRight(22)
                //.lineToConstantHeading(new Vector2d(10, 63 + Math.min(dsParams.x_LF, 3)))
                //prev 4
                //.lineToLinearHeading(dropPose[5])

                //.splineToLinearHeading(dropPose[5], Math.toRadians(180 + 67.5))
/*
                .waitSeconds(0.2)
                .addTemporalMarker(() -> {
                    slideDriver.dropGameElement();
                })
                .waitSeconds(0.1)
                .addTemporalMarker(() -> {
                    moveSlideToPos(3, SlideDirection.IN);
                    //sideArmDriver.activateSideArms(RobotSideArmDriver.RobotSideArmPreSetPos.DOWN);

                */
                //new Code
                //.waitSeconds(0.3)
                /*
                .addTemporalMarker(() -> {
                    inTakeDriver.intakeSet(true, false);
                })
                .waitSeconds(0.2)
                //.back(10)
                //.strafeLeft(3)
                .setTangent(Math.toRadians(180))

                .addTemporalMarker(() -> {
                    moveSlideToPos(3, SlideDirection.OUT);
                })

                .lineToConstantHeading(new Vector2d(4, 63 + dsParams.x_LS-0.7))
                //prev 14

                .lineToLinearHeading(dropPose[5])
                .addTemporalMarker(() -> {
                    inTakeDriver.intakeSet(false, false);
                })
                //.splineToLinearHeading(dropPose[4], Math.toRadians(180 + 67.5))

                //.waitSeconds(0.2)
                .addTemporalMarker(() -> {
                    slideDriver.dropGameElement();
                })

                .waitSeconds(0.1)
                .addTemporalMarker(() -> {
                    moveSlideToPos(3, SlideDirection.IN);
                    //sideArmDriver.activateSideArms(RobotSideArmDriver.RobotSideArmPreSetPos.DOWN);
                })
                .setVelConstraint( new MinVelocityConstraint( Arrays.asList(new AngularVelocityConstraint(DriveConstants.MAX_ANG_VEL ), new MecanumVelocityConstraint( 70, DriveConstants.TRACK_WIDTH ) ) ) )
                .lineToLinearHeading( new Pose2d( 10, wallPos+2 , Math.toRadians( 0 ) ))
                .forward(30)
                */
                .build();
                robot.followTrajectorySequence(moveToDropDuck1);
               /*
        dsParams = sensorDriver.getRobotSensorParams();
        TrajectorySequence finalPark = getTrajectorySequenceBuilder()

                //.strafeLeft(3)
                .build();
        robot.followTrajectorySequence(finalPark);

                 */
//        TrajectorySequence moveToPark = getTrajectorySequenceBuilder()
//                .setTangent(Math.toRadians(75))
//                //.splineToSplineHeading(new Pose2d(14, wallPos, Math.toRadians(0)), Math.toRadians(0))
//                //.lineToConstantHeading(new Vector2d(42, 63))
//                .setVelConstraint( new MinVelocityConstraint( Arrays.asList(new AngularVelocityConstraint(DriveConstants.MAX_ANG_VEL ), new MecanumVelocityConstraint( 70, DriveConstants.TRACK_WIDTH ) ) ) )
//                .lineToLinearHeading( new Pose2d( 4, wallPos , Math.toRadians( 0 ) ))
//
//                .strafeLeft(3)
//                .forward(38)
//                .addTemporalMarker(() -> {
//                    inTakeDriver.intakeSet(true, true);
//                })
//
//                .build();
//
//        robot.followTrajectorySequence(moveToPark);
//
//
//        ElapsedTime stopTime = new ElapsedTime();
//
//        while (opModeIsActive() && !isStopRequested() && (stopTime.seconds() < 3)) {
//            if (inTakeDriver.isElementCollected()) {
//                //Game Elemented is collected
//                break;
//            }
//
//            TrajectorySequence getDuck = getTrajectorySequenceBuilder()
//                    .forward(9)
//                    .build();
//            robot.followTrajectorySequence(getDuck);
//

            stopThreads();

        }
    }
