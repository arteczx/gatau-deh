package jp.jaxa.iss.kibo.rpc.flyoverspace;

import gov.nasa.arc.astrobee.Kinematics;
import gov.nasa.arc.astrobee.Result;
import gov.nasa.arc.astrobee.types.Point;
import gov.nasa.arc.astrobee.types.Quaternion;
import org.opencv.core.Mat;
import java.util.Map;
import java.util.List;

/**
 * Interface for the Treasure Hunt Game API
 */
public interface ApiService {
    /**
     * Get the current robot kinematics (position and orientation)
     */
    Kinematics getRobotKinematics();
    
    /**
     * Move to a specific point and orientation
     */
    Result moveTo(Point point, Quaternion quaternion, boolean b);
    
    /**
     * Move relative to the current position
     */
    void relativeMoveTo(Point point, Quaternion quaternion, boolean b);
    
    /**
     * Set information about an area (landmark or treasure)
     */
    void setAreaInfo(int index, String info, int number);
    
    /**
     * Control the front flashlight
     */
    void flashlightControlFront(float power);
    
    /**
     * Control the back flashlight
     */
    void flashlightControlBack(float power);
    
    /**
     * Get an image from the navigation camera
     */
    Mat getMatNavCam();
    
    /**
     * Save an image with a tag
     */
    void saveMatImage(Mat image, String tag);
    
    /**
     * Notify that an item has been recognized
     */
    void notifyRecognitionItem();
    
    /**
     * Take a snapshot of the target treasure
     */
    void takeTargetItemSnapshot();
    
    /**
     * Report completion of the mission
     */
    void reportRoundingCompletion();
    
    /**
     * Shutdown the factory
     */
    void shutdownFactory();
    
    /**
     * Move to a point with retry logic
     */
    void moveToWithRetry(Point point, Quaternion quaternion);
    
    /**
     * Detect ArUco markers in an image
     */
    List<Integer> detectArUcoMarkers(Mat image, String debugTag);
    
    /**
     * Create a quaternion for scanning a target
     */
    Quaternion createScanningQuaternion(Point target);
    
    /**
     * Create an optimized quaternion for scanning a target
     */
    Quaternion createOptimizedScanningQuaternion(Point target);
    
    /**
     * Check if a movement is valid
     */
    boolean isValidMovement(Point point, Quaternion quat);
    
    /**
     * Get the image cache
     */
    Map<String, Mat> getImageCache();
    
    /**
     * Get the map of found items
     */
    Map<Integer, String> getFoundItemsMap();
    
    /**
     * Internal method to start the mission
     */
    void internalStartMission();
    
    /**
     * Internal method to handle emergency situations
     */
    void internalHandleEmergency(); 
    
    /**
     * Internal method to initialize the camera
     */
    void internalInitCamera();
    
    /**
     * Internal method to handle mission failure
     */
    void internalHandleFailure();
    
    /**
     * Internal method to clean up resources
     */
    void internalCleanup();
    
    /**
     * Clean the image cache
     */
    void cleanImageCache();
    
    /**
     * Acquire a Mat from the pool
     */
    Mat acquireMatFromPool();
    
    /**
     * Release a Mat back to the pool
     */
    void releaseMatToPool(Mat mat);
    
    /**
     * Clear the Mat pool
     */
    void clearMatPool();
} 