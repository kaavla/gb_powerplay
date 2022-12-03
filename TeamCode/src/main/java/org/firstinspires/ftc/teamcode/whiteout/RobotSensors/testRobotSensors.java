package org.firstinspires.ftc.teamcode.whiteout.RobotSensors;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "testRobotSensors", group = "Test - TATA")
//@Disabled
public class testRobotSensors extends LinearOpMode {
    public RobotSensorDriver sensorDriver;
    public RobotSensorParams params;

    void printDebug(){
        params = sensorDriver.getRobotSensorParams();
        telemetry.addData("Front:", "%2f : %2f inc ", params.x_LF, params.x_RF);
        telemetry.addData("Rear:", "%2f : %2f inc ", params.x_LR, params.x_RR);
        telemetry.addData("Side (L-R):", "%2f : %2f inc ", params.x_LS, params.x_RS);
        switch (params.c_LS) {
            case RED:
                telemetry.addData("Color:", "Left Red");
                break;
            case BLUE:
                telemetry.addData("Color:", "Left Blue");
                break;
            case WHITE:
                telemetry.addData("Color:", "Left white");
                break;
            default:
                telemetry.addData("Color:", "No Color");


        }
        switch (params.c_RS) {
            case RED:
                telemetry.addData("Color:", "Right Red");
                break;
            case BLUE:
                telemetry.addData("Color:", "Right Blue");
                break;
            case WHITE:
                telemetry.addData("Color:", "Right white");
                break;
            default:
                telemetry.addData("Color:", "No Color");

        }

        telemetry.update();

    }
    @Override
    public void runOpMode() {

        sensorDriver = new RobotSensorDriver(hardwareMap, 100, telemetry);
        Thread sensorDriverThread = new Thread(sensorDriver);
        sensorDriverThread.start();

        double inc = 0.5;

        // Wait for the game to begin
        telemetry.addData(">", "Press Play to start op mode");
        telemetry.update();
        waitForStart();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            printDebug();
            sleep(500);
            idle();
        }
        sensorDriver.stop();
    }
}


