package jp.jaxa.iss.kibo.rpc.flyoverspace;

import gov.nasa.arc.astrobee.Kinematics;
import gov.nasa.arc.astrobee.Result;
import gov.nasa.arc.astrobee.types.Point;
import gov.nasa.arc.astrobee.types.Quaternion;
import org.opencv.core.Mat;
import java.util.Map;
import java.util.List;

public interface ApiService {
    Kinematics getRobotKinematics();
    Result moveTo(Point point, Quaternion quaternion, boolean b);
    void relativeMoveTo(Point point, Quaternion quaternion, boolean b);
    void setAreaInfo(int index, String info, int number);
    void flashlightControlFront(float power);
    void flashlightControlBack(float power);
    Mat getMatNavCam();
    void saveMatImage(Mat image, String tag);
    void notifyRecognitionItem();
    void takeTargetItemSnapshot();
    void reportRoundingCompletion();
    void shutdownFactory();
    void moveToWithRetry(Point point, Quaternion quaternion);
    List<Integer> detectArUcoMarkers(Mat image, String debugTag);
    Quaternion createScanningQuaternion(Point target);
    Quaternion createOptimizedScanningQuaternion(Point target);
    boolean isValidMovement(Point point, Quaternion quat);
    Map<String, Mat> getImageCache();
    Map<Integer, String> getFoundItemsMap();
    void internalStartMission();
    void internalHandleEmergency(); 
    void internalInitCamera();
    void internalHandleFailure();
    void internalCleanup();
    void cleanImageCache();
    Mat acquireMatFromPool();
    void releaseMatToPool(Mat mat);
    void clearMatPool();
} 