package org.firstinspires.ftc.teamcode.tata.Manual;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.trajectory.constraints.AngularVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.MecanumVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.MinVelocityConstraint;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.tata.Common.PoseStorage;
import org.firstinspires.ftc.teamcode.tata.Common.tataAutonomousBase;
import org.firstinspires.ftc.teamcode.tata.Common.tataMecanumDrive;
import org.firstinspires.ftc.teamcode.tata.RobotArm.RobotArmDriver;
import org.firstinspires.ftc.teamcode.tata.RobotSensors.RobotSensorParams;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

import java.util.Arrays;

@TeleOp(name = "Manual Mode", group = "Linear Opmode")

public class ManualDrive extends tataAutonomousBase {

    public void autoRunToPos() {

        Pose2d estimatedStartPose = new Pose2d(18, 63, Math.toRadians(0));
        robot.setPoseEstimate(estimatedStartPose);


        Pose2d pose = new Pose2d( -10,50  , Math.toRadians(82) );

        TrajectorySequence dropElement = getTrajectorySequenceBuilder()
                .setVelConstraint( new MinVelocityConstraint( Arrays.asList(new AngularVelocityConstraint( 60 ), new MecanumVelocityConstraint( 70, 14.1 ) ) ) )

                .addTemporalMarker(() -> {
                    //custommoveSlideToPos(3, SlideDirection.OUT, 20.5, 0.0);
                })

                .setTangent( Math.toRadians( 190) )
                .splineToLinearHeading( pose, Math.toRadians(262 ) )

                .build();
        robot.followTrajectorySequence(dropElement);

    }

    public void autoRunToSharedHub() {

        Pose2d estimatedStartPose = new Pose2d(63, 38, Math.toRadians(90));
        robot.setPoseEstimate(estimatedStartPose);


        Pose2d pose = new Pose2d( 59.9,13.5 , Math.toRadians(45) );

        TrajectorySequence dropElement = getTrajectorySequenceBuilder()
                .addTemporalMarker(() -> {
                    //custommoveSlideToPos(3, SlideDirection.OUT, 8.0, 0.0);
                })

                .back(18)
                .setTangent( Math.toRadians( 270-45) )
                .splineToLinearHeading(pose, Math.toRadians(270-45 ) )

                .build();
        robot.followTrajectorySequence(dropElement);

    }


    public void autoRunToWall() {

        Pose2d pose = new Pose2d(0, 64, Math.toRadians(0));

        TrajectorySequence dropElement = getTrajectorySequenceBuilder()
                .forward(4)
                .addTemporalMarker(() -> {
                    //custommoveSlideToPos(3, SlideDirection.IN, 20.5, 0.0);
                })

                //.setTangent( Math.toRadians( 90) )
                //.splineToSplineHeading(pose, Math.toRadians( 10) )
                .lineToLinearHeading(pose)

                .build();
        robot.followTrajectorySequence(dropElement);

    }
    public void autoRunToWallFromSharedHub() {

        Pose2d pose = new Pose2d(63,24 , Math.toRadians(90));

        TrajectorySequence dropElement = getTrajectorySequenceBuilder()
                .addTemporalMarker(() -> {
                    //custommoveSlideToPos(3, SlideDirection.IN, 8.0, 0.0);
                })
                .setTangent( Math.toRadians( 45) )
                .splineToLinearHeading(pose, Math.toRadians(90))
                .forward(24)

                .build();
        robot.followTrajectorySequence(dropElement);

    }


    @Override
    public void runOpMode() throws InterruptedException {
        //Blue
        //Pose2d startPose = new Pose2d(6, 61, Math.toRadians(270));

        //Red
        //Pose2d startPose = new Pose2d(6, -61, Math.toRadians(270));

        //PoseStorage.startPose = startPose;

        init(hardwareMap, PoseStorage.startPose, opModeCalled.MANUAL);
        robot.setPoseEstimate( PoseStorage.currentPose);

        waitForStart();

        if (isStopRequested()) {
            stopThreads();
            return;
        }


        while (!isStopRequested()) {
            robot.setWeightedDrivePower(
                    new Pose2d(
                            -gamepad1.left_stick_y/2,
                            -gamepad1.left_stick_x/2,
                            -gamepad1.right_stick_x/2
                    )
            );
            if (gamepad1.x) {
                autoRunToPos();
            } else if (gamepad1.y) {
                autoRunToWall();
            }
            /*else if (gamepad1.a) {
                autoRunToSharedHub();
            } else if (gamepad1.b) {
                autoRunToWallFromSharedHub();
            }*/

            inTakeDriver.checkGamePad(gamepad1);
            frDriver.checkGamePad(gamepad1);

            slideDriver.checkGamePadX(gamepad1); //only to rotate the arm

            slideDriver.checkGamePad(gamepad2);
            //armDriver.checkGamePad(gamepad2);
            crDriver.checkGamePad(gamepad2);
            frDriver.checkGamePad(gamepad1);

            idle();
            robot.update();

        }
    }


       // stopThreads();

    }
//}
