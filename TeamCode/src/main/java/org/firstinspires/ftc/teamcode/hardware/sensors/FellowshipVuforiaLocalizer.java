package org.firstinspires.ftc.teamcode.hardware.sensors;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;
import org.firstinspires.ftc.teamcode.R;

import java.util.ArrayList;
import java.util.List;

/**
 * Created by thomaslauer on 12/10/16.
 */
public class FellowshipVuforiaLocalizer extends FellowshipSensor {

    VuforiaLocalizer vuforia;

    VuforiaTrackables FTC_Trackables;
    List<VuforiaTrackable> allTrackables;

    OpenGLMatrix phoneLocation;
    OpenGLMatrix lastLocation = OpenGLMatrix.translation(0, 0, 0);

    @Override
    public void initialize(LinearOpMode opMode) {
        super.initialize(opMode);

        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters(R.id.cameraMonitorViewId);
        parameters.vuforiaLicenseKey = "AZrVU7T/////AAAAGVo9hsImhE6RojK+5tOA/zEuh6SPnDmpFUC14U9v2xbapUtN8fWjT8/cjuJjqybmMknEdiy5uP153iKIS5Bh8NmtymZrpVxH92vqmR7tvtEV/i2VcZBI6rwd181sRIdgphcr/vm4Ow5MoxqhSsBqXYXdElfMiINTfv2riOQsnnTqtMzDo3ZRczpK4rOtqHuSJ4zqrQcP5wJiJXGYGEMzfyryC1i3bMQuwZ7EFIVpCRFilct/s+N27b+gjSMwmvaIXGfU/Mmv4XCGuUZPLEi3pbXKix98RGNfgD4+L9m8qejf3bc7fqq4k3EDunBxAJp7oGq3mzuOTnaEu2L65QujzAlqTNPyTNDZynZshmcyLFlj";
        parameters.cameraDirection = VuforiaLocalizer.CameraDirection.FRONT;
        vuforia = ClassFactory.createVuforiaLocalizer(parameters);

        FTC_Trackables = vuforia.loadTrackablesFromAsset("FTC_2016-17");

        VuforiaTrackable targetWheels = FTC_Trackables.get(0);
        VuforiaTrackable targetTools = FTC_Trackables.get(1);
        VuforiaTrackable targetLegos = FTC_Trackables.get(2);
        VuforiaTrackable targetGears = FTC_Trackables.get(3);

        targetWheels.setName("Wheels");
        targetTools.setName("Tools");
        targetLegos.setName("Legos");
        targetGears.setName("Gears");

        OpenGLMatrix wheelsLocation = OpenGLMatrix
                .translation(0, 0, 0)
                .multiplied(Orientation.getRotationMatrix(
                        AxesReference.EXTRINSIC, AxesOrder.XZX,
                        AngleUnit.DEGREES, 90, 0, 0));
        targetWheels.setLocation(wheelsLocation);

        OpenGLMatrix toolsLocation = OpenGLMatrix
                .translation(0, 0, 0)
                .multiplied(Orientation.getRotationMatrix(
                        AxesReference.EXTRINSIC, AxesOrder.XZX,
                        AngleUnit.DEGREES, 90, 0, 0));
        targetTools.setLocation(toolsLocation);

        OpenGLMatrix legosLocation = OpenGLMatrix
                .translation(0, 0, 0)
                .multiplied(Orientation.getRotationMatrix(
                        AxesReference.EXTRINSIC, AxesOrder.XZX,
                        AngleUnit.DEGREES, 90, 0, 0));
        targetLegos.setLocation(legosLocation);

        OpenGLMatrix gearsLocation = OpenGLMatrix
                .translation(0, 0, 0)
                .multiplied(Orientation.getRotationMatrix(
                        AxesReference.EXTRINSIC, AxesOrder.XZX,
                        AngleUnit.DEGREES, 90, 0, 0));
        targetGears.setLocation(gearsLocation);

        phoneLocation = OpenGLMatrix
                .translation(0, 0, 0)
                .multiplied(OpenGLMatrix.rotation(
                        AxesReference.EXTRINSIC, AxesOrder.XYZ,
                        AngleUnit.DEGREES, 90, 0, 180));


        ((VuforiaTrackableDefaultListener) targetWheels.getListener()).setPhoneInformation(phoneLocation, parameters.cameraDirection);
        ((VuforiaTrackableDefaultListener) targetTools.getListener()).setPhoneInformation(phoneLocation, parameters.cameraDirection);
        ((VuforiaTrackableDefaultListener) targetLegos.getListener()).setPhoneInformation(phoneLocation, parameters.cameraDirection);
        ((VuforiaTrackableDefaultListener) targetGears.getListener()).setPhoneInformation(phoneLocation, parameters.cameraDirection);


        allTrackables = new ArrayList<VuforiaTrackable>();
        allTrackables.addAll(FTC_Trackables);
    }

    public void startTracking() {
        FTC_Trackables.activate();
    }

    public void stopTracking() {
        FTC_Trackables.deactivate();
    }

    public OpenGLMatrix updatePosition() {
        for (VuforiaTrackable trackable : allTrackables) {
            mOpMode.telemetry.addData(trackable.getName(),
                    ((VuforiaTrackableDefaultListener) trackable.getListener()).isVisible()
                            ? "visible" : "not visible");

            OpenGLMatrix robotLocationTransform =
                    ((VuforiaTrackableDefaultListener) trackable.getListener()).getUpdatedRobotLocation();

            if (robotLocationTransform != null) {
                lastLocation = robotLocationTransform;
            }
        }

        return lastLocation;
    }

    public boolean isTracked() {
        for (VuforiaTrackable trackable : allTrackables) {
            if(((VuforiaTrackableDefaultListener) trackable.getListener()).isVisible()) {
                return true;
            }
        }
        return false;
    }

    public double getTranslationX() {
        VectorF vec = lastLocation.getTranslation();
        return vec.get(0);
    }

    public double getTranslationY() {
        VectorF vec = lastLocation.getTranslation();
        return vec.get(1);
    }

    public double getTranslationZ() {
        VectorF vec = lastLocation.getTranslation();
        return vec.get(2);
    }

    public double getRotation() {
        return Math.atan2(getTranslationX(), getTranslationY());
    }
}
