package org.firstinspires.ftc.teamcode.whiteout.Blue;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.whiteout.Red.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.whiteout.Common.tataAutonomousBase;
import org.firstinspires.ftc.teamcode.whiteout.Common.tataMecanumDrive;

@Autonomous(group = "robot")
public class warehousebluepark extends tataAutonomousBase {
    public Pose2d startPose = new Pose2d(53.75, 66.25, Math.toRadians(180));//0 and 180 degrees

    @Override
    public void runOpMode() throws InterruptedException {

        Trajectory traj0, traj1, traj2, traj3, traj4;
        init(hardwareMap, startPose, opModeCalled.AUTO);
        robot.setPoseEstimate(startPose);

        waitForStart();

        if (isStopRequested()) {
            stopThreads();
            return;
        }

        //go forward
        traj0 = robot.trajectoryBuilder((startPose))
                .strafeLeft(5)
                .build();
        robot.followTrajectory(traj0);

        traj1 = robot.trajectoryBuilder((traj0.end()))
                .forward(14, tataMecanumDrive.getVelocityConstraint(15, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        tataMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .build();
        robot.followTrajectory(traj1);

        crDriver.toggleCarousel(false);
        sleep(5000);
        crDriver.toggleCarousel(false);

        traj2 = robot.trajectoryBuilder((traj1.end()))
                .back(55)
                .build();
        robot.followTrajectory(traj2);

        traj3 = robot.trajectoryBuilder((traj2.end()))
                .strafeRight(5.5)
                .build();
        robot.followTrajectory(traj3);

        traj4 = robot.trajectoryBuilder((traj3.end()))
                .back(40,tataMecanumDrive.getVelocityConstraint(15, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        tataMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .build();
        robot.followTrajectory(traj4);
    }
}

