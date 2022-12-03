package org.firstinspires.ftc.teamcode.whiteout.RobotLinearActuators;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "testLinearActuatorServo", group = "Test - TATA")
//@Disabled
public class testLinearActuatorServo extends LinearOpMode {
    public RobotLinearActuatorDriver driver0;
    public RobotLinearActuatorDriver driver1;

    @Override
    public void runOpMode() {
        driver0 = new RobotLinearActuatorDriver(hardwareMap, 1000, 0);
        Thread driverThread0 = new Thread(driver0);
        driverThread0.start();

        driver1 = new RobotLinearActuatorDriver(hardwareMap, 1000, 1);
        Thread driverThread1 = new Thread(driver1);
        driverThread1.start();

        // Wait for the game to begin
        telemetry.addData(">", "Press Play to start op mode");
        telemetry.update();
        waitForStart();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            driver0.pullLinearActuatorBy(-0.2);
            driver1.pullLinearActuatorBy(-0.2);

            sleep(5000);

            driver0.pullLinearActuatorBy(0.2);
            driver1.pullLinearActuatorBy(0.2);

            idle();
        }
        driver0.stop();
        driver1.stop();

    }
}

