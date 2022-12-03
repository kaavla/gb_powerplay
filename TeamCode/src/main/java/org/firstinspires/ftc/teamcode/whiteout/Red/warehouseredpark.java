package org.firstinspires.ftc.teamcode.whiteout.Red;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.whiteout.Common.tataAutonomousBase;

@Autonomous(group = "robot")
public class warehouseredpark extends tataAutonomousBase {
    public Pose2d startPose = new Pose2d(0, -66.25, Math.toRadians(180));

    @Override
    public void runOpMode() throws InterruptedException {

        Trajectory traj0;
        init(hardwareMap, startPose, opModeCalled.AUTO);
        robot.setPoseEstimate(startPose);

        waitForStart();

        if (isStopRequested()) {
            stopThreads();
            return;
        }

        //go forward

        traj0 = robot.trajectoryBuilder((startPose))
                .forward(48)
                .build();
        robot.followTrajectory(traj0);


    }
}