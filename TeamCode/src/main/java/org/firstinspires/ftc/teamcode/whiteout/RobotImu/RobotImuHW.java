package org.firstinspires.ftc.teamcode.whiteout.RobotImu;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;


public class RobotImuHW {

    // The IMU sensor object
    private BNO055IMU imu;

    // State used for updating telemetry
    private Orientation angles;
    private Acceleration gravity;

    private double rawHeading      = 0.0;
    private double relativeHeading = 0.0;
    private double correction      = 0.0;

    /* Initialize standard Hardware interfaces */
    public void init(HardwareMap ahwMap, double refAngle) {

        // Set up the parameters with which we will use our IMU. Note that integration
        // algorithm here just reports accelerations to the logcat log; it doesn't actually
        // provide positional information.
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        //parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        parameters.loggingEnabled      = true;
        parameters.loggingTag          = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();

        // Retrieve and initialize the IMU. We expect the IMU to be attached to an I2C port
        // on a Core Device Interface Module, configured to be a sensor of type "AdaFruit IMU",
        // and named "imu".
        imu = ahwMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);

        //This will be used to calculate offsets
        double initAngle = readImu();
        correction = refAngle - initAngle;
        if (correction < 0) {
            correction = 360 + correction;
        }
        RobotLog.ii("SHANK", "Imu Init %2f:%2f:%2f", initAngle, refAngle, correction);

    }

    public double readImu(){
        // Acquiring the angles is relatively expensive; we don't want
        // to do that in each of the three items that need that info, as that's
        // three times the necessary expense.
        angles   = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        rawHeading = angles.firstAngle;

        //raw heading is -180 to 180. So we convert to 0 to 359 scale
        if (rawHeading < 0)
            rawHeading += 360;

        return rawHeading;
    }

    public double getRelativeHeading(){
        double rel = rawHeading + correction;
        if (rel >= 360) {
            rel = rel - 360;
        }
        return rel;
    }

    public double getRawHeading(){
        return rawHeading;
    }

    public double getCorrection(){
        return correction;
    }


}

