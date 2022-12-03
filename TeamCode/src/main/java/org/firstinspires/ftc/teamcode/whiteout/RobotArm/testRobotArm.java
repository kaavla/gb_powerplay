package org.firstinspires.ftc.teamcode.whiteout.RobotArm;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "testRobotArm", group = "Test - TATA")
//@Disabled
public class testRobotArm extends LinearOpMode {
    public RobotArmDriver armDriver;
    public float leftX, leftY, rightZ;
    public RobotArmParams params;
    @Override
    public void runOpMode() {

        armDriver = new RobotArmDriver(hardwareMap, 50);
        Thread armDriverThread = new Thread(armDriver);
        armDriverThread.start();

        double inc = 0.5;

        // Wait for the game to begin
        telemetry.addData(">", "Press Play to start op mode");
        telemetry.update();
        waitForStart();

        //armDriver.initArmPos(0);
        double posIncr = 0.05;
        double newPos = 0.5;
        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            armDriver.checkGamePad(gamepad1);
            params = armDriver.getRobotParams();
            telemetry.addData("X, y:", "%2f, %2f inch ", params.xpos, params.ypos);
            telemetry.addData("Base:", "%2f Deg ", params.baseServoAbsPosInDegree);
            telemetry.addData("Link:", "%2f Deg ", params.linkServoAbsPosInDegree);
            telemetry.addData("Orie:", "%2f Deg, %2d ",
                    params.currOrientationInDegree,
                    params.motorEncoderRawVal);
            telemetry.addData("raw:", "%2f:%2f:%2f  ",
                    params.s1ServoRawPos,
                    params.s2ServoRawPos,
                    params.s3ServoRawPos);

            telemetry.update();
            sleep(50);
            idle();
        }
        armDriver.stop();
    }
}

