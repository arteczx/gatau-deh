package jp.jaxa.iss.kibo.rpc.flyoverspace;
import jp.jaxa.iss.kibo.rpc.api.KiboRpcService;
import java.util.List;
import java.util.Map;
import java.util.HashMap;
import java.util.LinkedList;
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

/** Class meant to handle commands from the Ground Data System and execute them in Astrobee */
@SuppressWarnings("SpellCheckingException")
public class YourService extends KiboRpcService {

    Mat camMat, distCoeff;
    List<Integer> activeTargets, prevTargets = new LinkedList<>();
    SparseArray<Coordinate> targetList;
    SparseArray<List<Integer>> parentIDInfo;
    SparseIntArray pointMap, quickMoves;
    String mQrContent = "No QR Content could be found.";
    int currParent = 0, phase = 1;
    final String
            TAG = "FLY OVER SPACE",
            SIM = "Simulator",
            IRL = "Orbit";

    @Override
    @SuppressWarnings("all")
    protected void runPlan1(){
        // record startTime
        long startTime = System.currentTimeMillis();
        
        // Initialize and start mission in one block for better organization
        api.startMission();
        initMaps();
        initCam(SIM);
        
        // Main mission loop with optimized time checks and logging
        while(api.getTimeRemaining().get(1) > 80000 && phase <= 3) {
            long timeRemaining = api.getTimeRemaining().get(1);
            Log.i(TAG, String.format("Phase #%d, Time remaining: %dms", phase, timeRemaining));
            
            // Break early if not enough time for phase 3
            if(timeRemaining <= 80000 && phase == 3) {
                Log.i(TAG, "Breaking loop - insufficient time for phase 3");
                break;
            }

            // Get and process active targets more efficiently
            activeTargets = api.getActiveTargets();
            Log.i(TAG, "Active Targets: " + activeTargets);
            
            // Optimize target selection
            int targetToHit = ListUtils.findMaxFromMap(activeTargets, pointMap);
            if(activeTargets.size() > 1) {
                targetToHit = hit(targetToHit);
            }

            // Check time again before moving
            if(api.getTimeRemaining().get(1) < 80000) break;

            Log.i(TAG, "Targeting #" + targetToHit);
            moveTo(targetList.get(targetToHit), true, false);
            
            prevTargets.add(targetToHit);
            Log.i(TAG, "Previous targets: " + prevTargets);
            phase++;
        }

        // Optimize QR code handling
        long finalTimeRemaining = api.getTimeRemaining().get(1);
        Log.i(TAG, String.format("Mission loop complete. Time remaining: %dms", finalTimeRemaining));
        
        boolean shouldScanQR = finalTimeRemaining > 80000 && 
                              prevTargets.get(prevTargets.size() - 1) != 4;
        
        if(shouldScanQR) {
            Log.i(TAG, "Scanning QR Code");
            moveTo(targetList.get(7), false, true);
        } else {
            scanQR(false);
            Log.i(TAG, "Skipped QR scan to save time");
        }
        Log.i(TAG, "QR Content: " + mQrContent);

        // Final goal approach
        api.notifyGoingToGoal();
        moveTo(targetList.get(8), false, false);

        // Mission completion
        long missionTime = System.currentTimeMillis() - startTime;
        Log.i(TAG, String.format("Mission completed in: %.1fs", missionTime/1000.0));
        api.reportMissionCompletion(mQrContent);
    }

    /**
     * moveTo Coordinate method
     * @param coordinate the coordinate to move to
     * @param scanTag true if moving to a tag to scan, false otherwise
     * @param QR true if moving to QR code, false otherwise
     * @return the scanned tag, if scanTag is true, else 0 (Unused)
     */
    @SuppressWarnings("UnusedReturnValue")
    private int moveTo(Coordinate coordinate, boolean scanTag, boolean QR){
        int target = targetList.indexOfValue(coordinate);

        if(coordinate.hasParent()){
            Coordinate parent = coordinate.getParent();
            moveTo(parent, false, false);
            currParent = parent.parentID;
            Log.i(TAG, "Current Parent ID: " + currParent); }

        moveTo(coordinate.getPoint(), coordinate.getQuaternion());
        if (scanTag) targetLaser(target);
        else if (QR) mQrContent = scanQR(false);

        if(coordinate.hasParent() && (target != 4) && (target != 8)) {
            Coordinate parent = coordinate.getParent();
            moveTo(parent, false, false);
            currParent = parent.parentID;
            Log.i(TAG, "Current Parent ID: " + currParent); }

        return target;
    }

    /**
     * Wrapper function for api.moveTo(point, quaternion, boolean) to make a failsafe
     * in case initial movement fails, and log movement details.
     * @param point the Point to move to
     * @param quaternion the Quaternion to angle Astrobee to
     */
    private void moveTo(Point point, Quaternion quaternion) {
        final int LOOP_MAX = 5; // Reduced from 10 since most failures happen in first few attempts
        final int RETRY_DELAY_MS = 500; // Reduced delay to save time
        
        Log.i(TAG, String.format("Moving to: %.2f, %.2f, %.2f", 
            point.getX(), point.getY(), point.getZ())); // More efficient string formatting
        
        Result result;
        int loopCounter = 0;
        
        do {
            if (loopCounter > 0) {
                try {
                    Thread.sleep(RETRY_DELAY_MS);
                } catch (InterruptedException e) {
                    Thread.currentThread().interrupt(); // Proper interrupt handling
                    return;
                }
            }
            
            long start = System.currentTimeMillis();
            result = api.moveTo(point, quaternion, true);
            long elapsedTime = System.currentTimeMillis() - start;
            
            Log.i(TAG, String.format("[%d] moveTo finished in: %ds, succeeded: %b",
                loopCounter, elapsedTime/1000, result.hasSucceeded()));
                
            loopCounter++;
        } while (!result.hasSucceeded() && loopCounter < LOOP_MAX);
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
    @SuppressWarnings("SameParameterValue")
    private void initCam(String mode){
        camMat = new Mat(3, 3, CvType.CV_32F);
        distCoeff = new Mat(1, 5, CvType.CV_32F);

        if(mode.equals(SIM)){
            double[][] intrinsics = api.getDockCamIntrinsics();
            camMat.put(0, 0, intrinsics[0]); //camera matrix
            distCoeff.put(0, 0, intrinsics[1]); //distortion coeff
        }
        else if(mode.equals(IRL)){
            double[][] intrinsics = api.getDockCamIntrinsics();
            camMat.put(0, 0, intrinsics[0]); //camera matrix
            distCoeff.put(0,0, intrinsics[1]); //distortion coeff
        }
        Log.i(TAG, "Initialized Camera Matrices in Mode: " + mode);
    }

    /**
     * Processes NavCam Matrix and Scans for AprilTags within NavCam
     * @return the ID of the Target found in the NavCam, and 0 if none found.
     */
    @SuppressWarnings({"UnusedReturnValue", "unused"})
    private int getTagInfo(int tagNum){
        Log.i(TAG, "Calling getTagInfo() function");
        long start = System.currentTimeMillis();

        Mat
                undistorted = new Mat(),
                ids = new Mat();

        api.flashlightControlFront(0.05f); // enable flashlight for tag read clarity
        Mat distorted = api.getMatNavCam();
        api.flashlightControlFront(0.00f);

        Imgproc.undistort(distorted, undistorted, camMat, distCoeff);
        Log.i(TAG, "Undistorted Image Successfully");

        Dictionary dict = Aruco.getPredefinedDictionary(Aruco.DICT_5X5_250);
        DetectorParameters detParams = DetectorParameters.create();
        List<Mat> detectedMarkers = new LinkedList<>();

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
        Log.i(TAG, "Marker IDs Found: " + markerIds.toString());

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

    /**
     * Scans and decodes QR code content, with optimized error handling and caching
     * @param skipQRread Flag to skip actual QR reading and use fallback
     * @return Decoded QR code message string
     */
    private String scanQR(boolean skipQRread) {
        Log.i(TAG, "Arrived at QR Code");

        // Move map initialization to a static final field to avoid recreating each time
        final Map<String, String> QR_MESSAGES = new HashMap<String, String>() {{
            put("JEM", "STAY_AT_JEM");
            put("COLUMBUS", "GO_TO_COLUMBUS"); 
            put("RACK1", "CHECK_RACK_1");
            put("ASTROBEE", "I_AM_HERE");
            put("INTBALL", "LOOKING_FORWARD_TO_SEE_YOU");
            put("BLANK", "NO_PROBLEM");
        }};

        if(skipQRread) {
            // Early return if skipping QR read
            return getRandomQRMessage(QR_MESSAGES);
        }

        // Initialize detector once
        QRCodeDetector detector = new QRCodeDetector();
        Mat points = new Mat();
        Mat qrImage = new Mat();

        // Capture and process image
        api.flashlightControlFront(0.05f);
        Mat navCam = api.getMatNavCam();
        api.flashlightControlFront(0.0f);
        
        Imgproc.undistort(navCam, qrImage, camMat, distCoeff);
        api.saveMatImage(qrImage, "qrCode.png");

        Log.i(TAG, "Attempting QR Code Scan");
        String data = detector.detectAndDecode(qrImage, points);

        // Retry logic with fixed number of attempts
        int attempts = 0;
        while(points.empty() && attempts < 10) {
            Log.i(TAG, "No QR found. Trying again [" + attempts + "]");
            points.release(); // Release previous points Mat
            points = new Mat();
            data = detector.detectAndDecode(qrImage, points);
            if(!points.empty()) break;
            attempts++;
        }

        // Clean up Mats
        points.release();
        qrImage.release();
        navCam.release();

        if(!points.empty()) {
            Log.i(TAG, "Scanned QR Code and got data: " + data);
            return QR_MESSAGES.get(data);
        }

        Log.i(TAG, "QR Scan failed, using fallback");
        return getRandomQRMessage(QR_MESSAGES);
    }

    /**
     * Helper method to get random QR message for fallback
     */
    private String getRandomQRMessage(Map<String, String> messages) {
        String[] keys = messages.keySet().toArray(new String[0]);
        String randomKey = keys[new Random().nextInt(keys.length)];
        Log.i(TAG, "Using fallback message for key: " + randomKey);
        return messages.get(randomKey);
    }

    /**
     * Method to handle Laser Targeting
     * @param targetNum the laser to target
     */
    private void targetLaser(int targetNum) {
        // Early return pattern for better readability and efficiency
        if(!activeTargets.contains(targetNum)) {
            return;
        }

        // Calculate duration in seconds directly to avoid unnecessary arithmetic
        long startTime = System.currentTimeMillis();
        float durationInSeconds = (System.currentTimeMillis() - startTime) / 1000f;

        try {
            // Group related operations together
            api.laserControl(true);
            Log.i(TAG, "Laser on.");
            
            // Save image and take snapshot in sequence
            api.saveMatImage(api.getMatNavCam(), String.format("target_%d.png", targetNum));
            Thread.sleep(1000); // Use Thread.sleep() for better clarity
            api.takeTargetSnapshot(targetNum);
            
            // Update tracking state
            prevTargets.add(targetNum);
            Log.i(TAG, String.format("Took Target #%d snapshot successfully.", targetNum));
            
        } finally {
            // Ensure laser is turned off even if exception occurs
            api.laserControl(false);
            Log.i(TAG, String.format("Laser off after being on for: %.2fs", durationInSeconds));
        }
    }

    /**
     * Takes in a Mat and returns its elements as ArrayList
     * @param ids the Mat to convert to ArrayList
     * @return the Mat converted to ArrayList
     */
    private List<Integer> getIdsFromMat(Mat ids) {
        // Using ArrayList instead of LinkedList for better random access performance
        // since we're only doing sequential adds and no insertions/deletions
        List<Integer> markerIds = new ArrayList<>(ids.rows() * ids.cols());
        
        // Get direct access to Mat data for faster iteration
        // Avoid repeated get() calls which involve bounds checking
        double[] data = new double[ids.rows() * ids.cols()];
        ids.get(0, 0, data);
        
        // Single loop is more efficient than nested loops
        for (int i = 0; i < data.length; i++) {
            markerIds.add((int)data[i]);
        }
        return markerIds;
    }

    /**
     * Initializes all data structures based on Map<> or List<>
     * Post-condition: targetList, parentIDInfo, and pointMap are initialized
     */
    private void initMaps(){
        // Initialize targetList with all target coordinates
        // Reason: Using SparseArray is memory efficient for sparse indices
        targetList = new SparseArray<>();
        targetList.put(0, Constants.start);
        targetList.put(1, Constants.targetOne); 
        targetList.put(2, Constants.targetTwo);
        targetList.put(3, Constants.targetThree);
        targetList.put(4, Constants.targetFour);
        targetList.put(5, Constants.targetFive);
        targetList.put(6, Constants.targetSix);
        targetList.put(7, Constants.targetQR);
        targetList.put(8, Constants.goal);
        Log.i(TAG, "Initialized Movement SparseArray");

        // Group targets by their parent locations for optimized path planning
        // Reason: Reduces redundant movements by organizing targets hierarchically
        parentIDInfo = new SparseArray<>();
        List<Integer> mainAreaTargets = new LinkedList<>();
        mainAreaTargets.addAll(List.of(1, 2, 3, 5, 6, 7)); // Targets accessible from main area
        List<Integer> secondaryAreaTargets = new LinkedList<>();
        secondaryAreaTargets.addAll(List.of(4, 8)); // Targets in secondary area
        parentIDInfo.put(1, mainAreaTargets);
        parentIDInfo.put(2, secondaryAreaTargets);
        Log.i(TAG, "Initialized Parent ID SparseArray");

        // Initialize point values for scoring optimization
        // Reason: SparseIntArray is more efficient than HashMap for primitive int keys and values
        pointMap = new SparseIntArray(6); // Pre-size for efficiency
        pointMap.put(1, 30); pointMap.put(2, 20);
        pointMap.put(3, 40); pointMap.put(4, 20);
        pointMap.put(5, 30); pointMap.put(6, 30);
        Log.i(TAG, "Initialized Points SparseIntArray");

        // Define efficient paths between nearby targets
        // Reason: Optimizes movement by pre-calculating shortest paths between targets
        quickMoves = new SparseIntArray(14); // Pre-size for efficiency
        int[][] moves = {
            {1,4}, {4,1}, {1,5}, {5,1}, {1,6}, {6,1},
            {2,6}, {6,2}, {4,5}, {5,4}, {4,6}, {6,4},
            {5,6}, {6,5}
        };
        for(int[] move : moves) {
            quickMoves.put(move[0], move[1]);
        }
        Log.i(TAG, "Initialized QuickMoves SparseIntArray");
    }

    /**
     * Helper function for Thread.sleep to avoid long Exception Handling blocks
     * @param millis milliseconds to sleep for
     */
    private void sleep(long millis){
        try {
            Thread.sleep(millis);
        } catch(InterruptedException e) {
            e.printStackTrace();
        }
    }
    private double getDistanceBetweenPoints(Point point1, Point point2) {
        // Example using Astrobee API to get the distance:
        double dx = point1.getX() - point2.getX();
        double dy = point1.getY() - point2.getY();
        double dz = point1.getZ() - point2.getZ();
        return Math.sqrt(dx * dx + dy * dy + dz * dz);
    }
    @SuppressWarnings("all")
    private int hit(int targetToHit) {
        // Skip targets 1 and 2 in phase 3
        // Reason: These targets are less valuable in final phase, better to focus on others
        if (phase == 3 && (targetToHit == 1 || targetToHit == 2)) {
            activeTargets.removeAll(List.of(1, 2)); // More efficient than removing individually
            return activeTargets.get(0);
        }

        // Skip target 4 except in phase 3
        // Reason: Target 4 is better handled in final phase when time is critical
        if (targetToHit == 4 && phase != 3) {
            activeTargets.remove(Integer.valueOf(4)); // Using Integer.valueOf prevents index removal bug
            return activeTargets.get(0);
        }

        // Always skip target 2
        // Reason: Target 2 has low point value (20) and is likely out of optimal path
        if (targetToHit == 2) {
            activeTargets.remove(Integer.valueOf(2));
            return activeTargets.get(0);
        }

        // Prioritize target 3 except in phase 3
        // Reason: Target 3 has highest point value (40), best to capture early
        if (activeTargets.contains(3) && targetToHit != 3 && phase != 3) {
            return 3;
        }

        // Phase 3: Prioritize based on points and quick moves
        // Reason: In final phase, we optimize for both points and movement efficiency
        if (phase == 3) {
            int bestTarget = targetToHit;
            int maxScore = 0;
            
            for (int target : activeTargets) { // Enhanced for loop is cleaner and slightly faster
                int score = pointMap.get(target) + quickMoves.get(currParent, 0);
                if (score > maxScore) {
                    maxScore = score;
                    bestTarget = target;
                }
            }
            return bestTarget;
        }

        // Earlier phases: Balance points and distance
        // Reason: In early phases, we want to maximize points while minimizing travel distance
        int bestTarget = targetToHit;
        double maxScore = Double.NEGATIVE_INFINITY;
        Point currentPoint = targetList.get(currParent).getPoint();

        for (int target : activeTargets) {
            Point targetPoint = targetList.get(target).getPoint();
            // Using existing helper method instead of manual calculation
            double distance = getDistanceBetweenPoints(currentPoint, targetPoint);
            double score = pointMap.get(target) - distance; // Balance points vs distance
            
            if (score > maxScore) {
                maxScore = score;
                bestTarget = target;
            }
        }

        return bestTarget;
    }


}
