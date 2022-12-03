package org.firstinspires.ftc.teamcode.whiteout.RobotSideArm;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "testRobotSideArm", group = "Test - TATA")
//@Disabled
public class testRobotSideArm extends LinearOpMode {
    public RobotSideArmDriver sideArmDriver;
    @Override
    public void runOpMode() {

        sideArmDriver = new RobotSideArmDriver(hardwareMap, 50);
        Thread sideArmDriverThread = new Thread(sideArmDriver);
        sideArmDriverThread.start();


        // Wait for the game to begin
        telemetry.addData(">", "Press Play to start op mode");
        telemetry.update();
        waitForStart();

        //armDriver.initArmPos(0);
        double posIncr = 0.05;
        double newPos = 0.5;
        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            sideArmDriver.activateSideArms(RobotSideArmDriver.RobotSideArmSide.BOTH_SIDES, RobotSideArmDriver.RobotSideArmPreSetPos.DOWN);
            sleep(3000);
            sideArmDriver.activateSideArms(RobotSideArmDriver.RobotSideArmSide.BOTH_SIDES, RobotSideArmDriver.RobotSideArmPreSetPos.UP);
            sleep(3000);

            idle();
        }
        sideArmDriver.stop();
    }
}

