package org.firstinspires.ftc.teamcode.whiteout.RobotFrontServo;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "testRobotFrontServo", group = "Test - TATA")
//@Disabled
public class testRobotFrontServo extends LinearOpMode {
    public RobotFrontServoDriver driver;

    private double motor_power = 0.9;
    @Override
    public void runOpMode() {
        driver = new RobotFrontServoDriver(hardwareMap, 1000);
        Thread driverThread = new Thread(driver);
        driverThread.start();

        // Wait for the game to begin
        telemetry.addData(">", "Press Play to start op mode");
        telemetry.update();
        waitForStart();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            driver.checkGamePad(gamepad2);
            sleep(50);
            idle();
        }
        driver.stop();
    }
}

