package org.firstinspires.ftc.teamcode.whiteout.RobotSlide;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "testRobotSlide", group = "Test - TATA")
//@Disabled
public class testRobotSlide extends LinearOpMode {
    public RobotSlideDriver slideDriver;
    public float leftX, leftY, rightZ;
    public RobotSlideParams params;
    @Override
    public void runOpMode() {

        slideDriver = new RobotSlideDriver(hardwareMap, 50, true);
        Thread slideDriverThread = new Thread(slideDriver);
        slideDriverThread.start();

        double inc = 0.5;

        // Wait for the game to begin
        telemetry.addData(">", "Press Play to start op mode");
        telemetry.update();
        waitForStart();

        double posIncr = 0.05;
        double newPos = 0.5;
        // run until the end of the match (driver presses STOP)
        //while (opModeIsActive()) {
            //slideDriver.checkGamePad(gamepad2);
            slideDriver.moveRobotSlideBy(6,0);
            sleep(700);
            slideDriver.dropGameElement();
            sleep(2000);

        slideDriver.moveRobotSlideBy(-6,0);
            sleep(2000);


            params = slideDriver.getRobotSlideParams();
            telemetry.addData("Arm:", "%2f inc ", params.pos);
            telemetry.addData("Incl:", "%2f inc ", params.inclination);
            telemetry.addData("Enc:", "%2d,%2d    inc ",
                    params.encoderPosSl1,
                    params.encoderPosSl1);
            telemetry.addData("Servo:", "%2f,%2f ",
                    params.servo0Pos,
                    params.servo1Pos);
            telemetry.update();
            sleep(50);
            idle();
        //}
        slideDriver.stop();
    }
}

