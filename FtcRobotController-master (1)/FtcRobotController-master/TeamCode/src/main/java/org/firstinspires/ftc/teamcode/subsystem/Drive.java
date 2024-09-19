package org.firstinspires.ftc.teamcode.subsystem;

import android.os.Environment;

import com.arcrobotics.ftclib.command.OdometrySubsystem;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.drivebase.MecanumDrive;
import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.arcrobotics.ftclib.kinematics.HolonomicOdometry;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.firstinspires.ftc.vision.tfod.TfodProcessor;

import java.util.List;
import java.util.Locale;
import java.util.function.DoubleSupplier;

public class Drive extends SubsystemBase {
    MecanumDrive mecanumDrive;
    IMU gyro;
    double mult = 1;
    Telemetry telemetry;
    String mode = "Field";
    String speed = "Fast";
    MotorEx fLMotor, fRMotor, bLMotor, bRMotor;
    AprilTagProcessor aprilTag;
    TfodProcessor tensorFlow;
    VisionPortal visionPortal;
    List<AprilTagDetection> detections;
    List<Recognition> recognitions;
    String[] LABELS = {"crown"};
    String MODEL = Environment.getExternalStorageDirectory().getPath() + "/FIRST/tflitemodels/crown.tflite";
    HolonomicOdometry odometry;
    Motor.Encoder leftEncoder, rightEncoder, centerEncoder;
    double TICK_TO_INCH_ENC;
    double TRACK_WIDTH = 12.5;
    double WHEEL_OFFSET = -6.75;
    double area = 0;
    double turn = 0;
    double direction = 1;
    int desiredTagID;
    AprilTagDetection desiredTag = null;
    boolean atX = false;
    boolean atY = false;
    boolean atTag = false;
    boolean atTarget = false;
    public enum Zone {LEFT, CENTER, RIGHT}

    private Zone zone = Zone.LEFT;

    public Drive(HardwareMap hwMap, Telemetry telemetry) {
        IMU.Parameters parameters = new IMU.Parameters(
                new RevHubOrientationOnRobot(
                        RevHubOrientationOnRobot.LogoFacingDirection.RIGHT,
                        RevHubOrientationOnRobot.UsbFacingDirection.FORWARD)
        );

        gyro = hwMap.get(IMU.class, "imu");

        this.fLMotor = new MotorEx(hwMap, "flm");
        this.fRMotor = new MotorEx(hwMap, "frm");
        this.bLMotor = new MotorEx(hwMap, "blm");
        this.bRMotor = new MotorEx(hwMap, "brm");

        this.fLMotor.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        this.fRMotor.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        this.bLMotor.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        this.bRMotor.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);

        TICK_TO_INCH_ENC = (double) 3 * (48 / 25.4) / 2000;

        this.telemetry = telemetry;

        leftEncoder = bLMotor.encoder.setDistancePerPulse(TICK_TO_INCH_ENC);
        rightEncoder = fLMotor.encoder.setDistancePerPulse(TICK_TO_INCH_ENC);
        centerEncoder = bRMotor.encoder.setDistancePerPulse(TICK_TO_INCH_ENC);

        rightEncoder.setDirection(Motor.Direction.REVERSE);

        odometry = new HolonomicOdometry(leftEncoder::getDistance, rightEncoder::getDistance, centerEncoder::getDistance, TRACK_WIDTH, WHEEL_OFFSET);

        mecanumDrive = new MecanumDrive(fLMotor, fRMotor, bLMotor, bRMotor);

        aprilTag = new AprilTagProcessor.Builder().build();

        tensorFlow = new TfodProcessor.Builder()
                .setModelFileName(MODEL)
                .setModelLabels(LABELS)
                .build();

        visionPortal = new VisionPortal.Builder()
                .setCamera(hwMap.get(WebcamName.class, "Webcam 1"))
                .addProcessors(tensorFlow, aprilTag)
                .build();

        tensorFlow.setZoom(1);
        visionPortal.setProcessorEnabled(tensorFlow, true);
        visionPortal.setProcessorEnabled(aprilTag, false);

        tensorFlow.setMinResultConfidence(.35f);

        gyro.initialize(parameters);
    }

    @Override
    public void periodic() {

        getGeneralTelem();
        telemetry.addData("zone", checkZone());
        getAprilTagTelem();
        getTensorFlowTelem();


      telemetry.update();
    }

    /*public void updateOdom() {
        for (AprilTagDetection detection : detections) {
            if (detection.metadata != null) {
                if (detection.id == 7) {
                    double x = Constants.Tag7X + detection.ftcPose.x - Constants.CamX;
                    double y = Constants.Tag7Y + detection.ftcPose.y - Constants.CamY;
                    Rotation2d rx = new Rotation2d(getYaw());
                    odometry.updatePose(new Pose2d(x, y, rx));
                }
                if (detection.id == 10) {
                    double x = Constants.Tag10X + detection.ftcPose.x - Constants.CamX;
                    double y = Constants.Tag10Y + detection.ftcPose.y - Constants.CamY;
                    Rotation2d rx = new Rotation2d(getYaw());
                    odometry.updatePose(new Pose2d(x, y, rx));
                }
            } else odometry.updatePose();

        }
    }*/

    public void getGeneralTelem() {
        telemetry.addData("WantID", desiredTagID);
        DoubleSupplier cEnc = centerEncoder::getDistance;
        DoubleSupplier lEnc = leftEncoder::getDistance;
        DoubleSupplier rEnc = rightEncoder::getDistance;
        telemetry.addLine(mode + " Centric Drive");
        telemetry.addLine(speed);
        telemetry.addData("Yaw", getYaw());
        telemetry.addData("c", cEnc.getAsDouble());
        telemetry.addData("r", rEnc.getAsDouble());
        telemetry.addData("l", lEnc.getAsDouble());
        telemetry.addData("atX", atX);
        telemetry.addData("atY", atY);

        /*telemetry.addData("frdist", fRMotor.getCurrentPosition());
        telemetry.addData("fldist", fLMotor.getCurrentPosition());
        telemetry.addData("brdist", bRMotor.getCurrentPosition());
        telemetry.addData("bldist", bLMotor.getCurrentPosition());*/
       /* telemetry.addData("", this.fLMotor.get());
        telemetry.addData("", this.fRMotor.get());
        telemetry.addData("", this.bLMotor.get());
        telemetry.addData("", this.bRMotor.get());*/
    }

    public void getAprilTagTelem() {
        detections = aprilTag.getDetections();
        telemetry.addData("# AprilTags Detected", detections.size());
        for (AprilTagDetection detection : detections) {
            if (detection.metadata != null) {
                telemetry.addLine(String.format(Locale.US, "\n==== (ID %d) %s", detection.id, detection.metadata.name));
                telemetry.addLine(String.format(Locale.US, "XYZ %6.1f %6.1f %6.1f  (inch)", detection.ftcPose.x, detection.ftcPose.y, detection.ftcPose.z));
                telemetry.addLine(String.format(Locale.US, "PRY %6.1f %6.1f %6.1f  (deg)", detection.ftcPose.pitch, detection.ftcPose.roll, detection.ftcPose.yaw));
                telemetry.addLine(String.format(Locale.US, "RBE %6.1f %6.1f %6.1f  (inch, deg, deg)", detection.ftcPose.range, detection.ftcPose.bearing, detection.ftcPose.elevation));
            }
        }
    }

    public void getTensorFlowTelem() {
        recognitions = tensorFlow.getRecognitions();
        telemetry.addData("# Objects Detected", recognitions.size());
        for (Recognition recognition : recognitions) {
            double x = (recognition.getLeft() + recognition.getRight()) / 2;
            double y = (recognition.getTop() + recognition.getBottom()) / 2;
            telemetry.addData("", " ");
            telemetry.addData("Image", "%s (%.0f %% Conf.)", recognition.getLabel(), recognition.getConfidence() * 100);
            telemetry.addData("- Position", "%.0f / %.0f", x, y);
            telemetry.addData("- Size", "%.0f x %.0f", recognition.getWidth(), recognition.getHeight());

        }
        telemetry.update();
    }

    public Zone getZone() {
        zone = Zone.LEFT;
        recognitions = tensorFlow.getRecognitions();
        for (Recognition recognition : recognitions) {
            double x = (recognition.getLeft() + recognition.getRight()) / 2;
            if (recognitions.size() == 0) {
                zone = Zone.LEFT;
            } else if (x > 410) {
                zone = Zone.RIGHT;
            } else if (x > 40 && x < 410) {
                zone = Zone.CENTER;
            }
        }

        return zone;
    }

    public Zone checkZone(){
        return zone;
    }

    public double getYaw() {
        return gyro.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
    }

    public void robotCentricDrive(double x, double y, double rx) {
        mecanumDrive.driveRobotCentric(x * mult * direction, y * mult* direction, rx * mult*direction);
        mode = "Robot";
    }

    public void fieldCentricDrive(double x, double y, double rx) {
        mecanumDrive.driveFieldCentric(x * mult* direction, y * mult* direction, rx * mult* direction, getYaw());
        mode = "Field";
    }

    public void setMult(double m) {
        mult = m;
        if (mult < 1) speed = "Slow";
        else speed = "Fast";
    }
    public void setTurn(double t){
        turn = t;
    }
    public void setDirection(double d){
        direction = d;
    }

    public DoubleSupplier getLEnc() {
        return (leftEncoder::getDistance);
    }

    public DoubleSupplier getREnc() {
        return (rightEncoder::getDistance);
    }

    public DoubleSupplier getCEnc() {
        return (centerEncoder::getDistance);
    }

    public double getYEnc() {
        return (getREnc().getAsDouble() + getLEnc().getAsDouble()) / 2;
    }

    public void stop() {
        mecanumDrive.stop();
    }

    public void resetEncoders() {
        fLMotor.resetEncoder();
        fRMotor.resetEncoder();
        bLMotor.resetEncoder();
        bRMotor.resetEncoder();
    }
    public void resetGyro() {
        gyro.resetYaw();
    }

    public void camOff(){
        visionPortal.close();
    }




    public void setInRange(){
        double targetX = 0;
        double targetY = 0;
        double tolerance = 1;

        detections = aprilTag.getDetections();
        for (AprilTagDetection detection : detections) {

            if (detection.id == desiredTagID) {
                desiredTag = detection;
                atTag = true;
            }

             atX = Math.abs(Math.abs(desiredTag.ftcPose.x)-Math.abs(targetX)) < tolerance;
             atY = Math.abs(Math.abs(desiredTag.ftcPose.y)-Math.abs(targetY)) < tolerance;

            if ( atY && atX ){
                atTarget = true;
            }
        }
    }

    public boolean inRange(){
        return (atTarget && atTag);
    }
}
