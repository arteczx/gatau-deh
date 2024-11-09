package jp.jaxa.iss.kibo.rpc.flyoverspace;
import jp.jaxa.iss.kibo.rpc.api.KiboRpcService;

import java.util.List;
import java.util.Map;
import java.util.HashMap;
import java.util.ArrayList;
import java.util.Random;

import gov.nasa.arc.astrobee.Result;
import gov.nasa.arc.astrobee.types.Point;
import gov.nasa.arc.astrobee.types.Quaternion;

import org.opencv.core.CvType;
import org.opencv.core.Mat;

import org.opencv.aruco.Aruco;
import org.opencv.aruco.DetectorParameters;
import org.opencv.aruco.Dictionary;

import org.opencv.imgproc.Imgproc;
import org.opencv.objdetect.QRCodeDetector;

import android.util.Log;
import android.util.SparseArray;
import android.util.SparseIntArray;

/**
 * Class meant to handle commands from the Ground Data System and execute them in Astrobee.
 */
@SuppressWarnings("SpellCheckingException")
public class YourService extends KiboRpcService {

    Mat camMat, distCoeff;
    List<Integer> activeTargets, prevTargets = new ArrayList<>();
    SparseArray<Coordinate> targetList;
    SparseArray<List<Integer>> parentIDINfo;
    SparseArray pointMap, quickMoves;

    String mQrContent = "Content in scanned QR is not found";
    int currParent = 0, phase = 1;

    final String
            TAG = "FLY OVER SPACE",
            SIM = "Simulator",
            IRL = "Orbit"
    @Override
    @SuppressWarnings("all")
    protected void runPlan1(){
        //record time
        long startTime = System.currentTimeMillis();
        // The mission starts.
        api.startMission();

        //handle movemnt
        while(api.getTimeRemaining().get(1) > 80000 && phase <= 3) {
            Log.i(TAG, "Phase #"  + phase);
            int targetToHit = 0;

            if(api.getTimeRemaining().get(1) <= 80000 && phase == 3) {
                Log.i(TAG, "Breaking loop");
                break;
            }

            activeTargets = api.getActiveTargets();
            Log.i(TAG, "Active Targets: " + activeTargets.toString());

            targetToHit = ListUtils.findMaxFromMap(activeTargets, pointMap);
            if (activeTargets.size() > 1)
                targetToHit = HITTING(targetToHit);

            if(api.getTimeRemaining().get(1) < 80000) break;

            Log.i(TAG, "Going to target #" + targetToHit);
            moveTo(targetList.get(targetToHit), true, false);

            Log.i(TAG, "Time remain after target #" + targetToHit + ": " + api.getTimeRemaining().get(1) + "ms");
            Log.i (TAG, "prev targets: " + prevTargets.toString());
            phase++
        }
        Log.i(TAG, "Exit loop time: " + api.getTimeRemaining().get(1) + "ms remain");

        //qr
        if(prevTargets.get(prevTargets.size() - 1) != 4 && api.getTimeRemaining().get(1) > 80000) {
            Log.i(TAG, "Procced to QR Code");
            moveTo(targetList.get(7), false, true);
        } else {
            scanQR(false);
            Log.i(TAG, "Skip qr to prioritize time");
        }
        Log.i(TAG, "QR Content: " + mQrContent);

        api.notifyGoingToGoal();
        moveTo(targetList.get(8), false, false);

        //mission complete
        long deltaTime = System.currentTimeMillis() - startTime;
        Log.i(TAG, "plan 1 executed in: " + deltaTime/1000 + "second.");
        api.reportRoundingCompletion(mQrContent);
    }

    @Override
    protected void runPlan2(){// write your plan 2 here.
    }

    @Override
    protected void runPlan3(){// write your plan 3 here.
    }

    private int moveTo(Coordinate coordinate, boolean scanTag, boolean QR){
        int target = targetList.indexOfValue(coordinate);

        if(coordinate.hasParent()){
            Coordinate parent = coordinate.getParent();
            moveTo(parent, false, false);
            currParent = parent.parentID;
            Log.i(TAG, "Current Parr ID: " + currParent);
        }

        moveTo(coordinate.getPoint(), coordinate.getQuaternion());
        if (scanTag) targetLaser(target);
        else if (QR) mQrContent = scanQR(false);

        if(coordinate.hasParent() && (target != 4) && (target != 8)) {
            Coordinate parent = coordinate.getParent();
            moveTo(parent, false, false);
            currParent = parent.parentID;
            Log.i(TAG, "Current Parent ID: " + currParent);
        }

        return target;
    }
    /**
     * Wrapper function for api.moveTo(point, quaternion, boolean) to make a failsafe
     * in case initial movement fails, and log movement details.
     * @param point the Point to move to
     * @param quaternion the Quaternion to angle Astrobee to
     */

    private void moveTo(Point point, Quaternion quaternion) {
        final int LOOP_MAX = 10;

        Log.i(TAG, "moving to: " + point.getX() + ", " + point.getY() + "," + point.getZ());
        long start = System.currentTimeMillis();

        Result result = api.moveTo(point, quaternion, true);

        long end = System.currentTimeMillis();
        long elapsedTime = end - start;
        Log.i(TAG, "move finished in : " + elapsedTime/1000 + "seconds");
        Log.i(TAG, "hasSucceeded : " + result.hasSucceeded());

        int loopCounter = 1;
        while (!result.hasSucceeded() && loopCounter <= LOOP_MAX) {
            Log.i(TAG, "(" + loopCounter + ")" + "Calling moveTo func");
            start = System.currentTimeMillis();

            result = api.moveTo(point, quaternion, true);

            end = System.currentTimeMillis();
            elapsedTime = end -start;
            Log.i(TAG, "[" + loopCounter + "] " + "moveTo finished in : " + elapsedTime / 1000 +
                    " seconds");
            Log.i(TAG, "[" + loopCounter + "] " + "hasSucceeded : " + result.hasSucceeded());
            loopCounter++;
        }
    }

    /** moveTo double/float Specifics wrapper */
    @SuppressWarnings("unused")
    private void moveTo(double pt_x, double pt_y, double pt_z, float q_x, float q_y, float q_z, float q_w){
        moveTo(new Point(pt_x, pt_y, pt_z), new Quaternion(q_x, q_y, q_z, q_w));
    }

    /**
     * Pre-Load the Camera Matrix and Distortion Coefficients to save time.
     * @param mode 'SIM' or 'IRL' -> Simulator or Real Coefficients, Respectively
     */

    private void initCam(String mode){
        camMat = new Mat(3, 3, CvType.CV_32F);
        distCoeff = new Mat(1, 5, CvType.CV_32F);

        if(mode.equals(SIM)){
            //random coef
            float[] camArr = {
                    661.783002f, 0.000000f, 595.212041f,
                    0.000000f, 671.508662f, 489.094196f,
                    0.000000f, 0.000000f, 1.000000f
            };
            float[] distortionCoefficients = {
                    -0.215168f, 0.044354f, 0.003615f, 0.005093f, 0.000000f
            };

            camMat.put(0, 0, camArr);
            distCoeff.put(0, 0, distortionCoefficients);
        }
        else if(mode.equals(IRL)){
            float[] camArr = {
                    753.51021f, 0.0f, 631.11512f,
                    0.0f, 751.3611f, 508.69621f,
                    0.0f, 0.0f, 1.0f
            };
            float[] distortionCoefficients = {
                    -0.411405f, 0.177240f, -0.017145f, 0.006421f, 0.000000f
            };

            camMat.put(0, 0, camArr);
            distCoeff.put(0,0, distortionCoefficients);
        }
        Log.i(TAG, "Init Camera Matrix in mode: " + mode);
    }

    /**
     * proc NavCam Matrix and Scans for AprilTags within NavCam
     */
    private int getTagInfo(int tagNum){
        Log.i(TAG, "Call get tag info func");
        long start = System.currentTimeMillis();

        Mat
                undistorted = new Mat(),
                ids = new Mat();
        api.flashlightControlFront(0.05f);
        Mat distorted = api.getMatNavCam();
        api.flashlightControlFront(0.00f);

        Imgproc.undistort(distorted, undistorted, camMat, distCoeff);
        Log.i(TAG, "Undis Img success");

        Dictionary dict = Aruco.getPredefinedDictionary(Aruco.DICT_5X5_250);
        DetectorParameters detParams = DetectorParameters.create();
        List<Mat> detectedMarkers = new ArrayList<>();

        Aruco.detectMarkers(undistorted, dict, detectedMarkers, ids, detParams);
        List<Integer> markerIds = getIdsFromMat(ids);

        int iters = 0, iter_max = 10, target = 0;
        while(markerIds.size() == 0 && iters < iter_max){
            Log.i(TAG, "No Markers found. Trying again [" + iters + "]");
            Aruco.detectMarkers(undistorted, dict, detectedMarkers, ids, detParams);
            markerIds = getIdsFromMat(ids);
            if(markerIds.size() != 0)
                break;

            iters++;
        }
        Log.i(TAG, "MARKER ID FOUND: " + markerIds.toString());

        if (ListUtils.containsAny(markerIds, Constants.targetOneIDs)){target = 1;}
        else if (ListUtils.containsAny(markerIds, Constants.targetTwoIDs)){target = 2;}
        else if (ListUtils.containsAny(markerIds, Constants.targetThreeIDs)){target = 3;}
        else if (ListUtils.containsAny(markerIds, Constants.targetFourIDs)){target = 4;}
        else if (ListUtils.containsAny(markerIds, Constants.targetFiveIDs)){target = 5;}
        else if (ListUtils.containsAny(markerIds, Constants.targetSixIDs)){target = 6;}

        long delta = (System.currentTimeMillis() - start)/1000;
        Log.i(TAG, "Found Target #" + target);
        Log.i(TAG, "Read AprilTags in " + delta + " seconds");

        api.saveMatImage(undistorted, "tag" + target + "Image.png");
        return target;
    }
}
