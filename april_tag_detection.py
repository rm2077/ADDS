import cv2
import robotpy_apriltag as tag



def open_camera():
    cap = cv2.VideoCapture(1)
    
    ret, frame = cap.read()
    while cap.isOpened():
        ret, frame = cap.read()
        coords = []
        
        if frame.dtype != "uint8":
            frame = frame.astype("uint8")
        
        img = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        
        detector = tag.AprilTagDetector()
        detector.addFamily('tag25h9')
        detector.addFamily('tag36h11')
        detector.addFamily('tagcircle21h7')
        predictions = detector.detect(img)
        for pred in predictions:
            coords.append((pred.getCorner(3), pred.getCorner(1)))
            
        for coord in coords:
            cv2.rectangle(img, (int(coord[0].x), int(coord[0].y)), (int(coord[1].x), int(coord[1].y)), color=(34,34,34), thickness=2)
        
        cv2.imshow('frame', cv2.cvtColor(img, cv2.COLOR_BGR2RGB))
        
        if cv2.waitKey(1) & 0xFF == ord("q"):
            break

    cap.release()
    cv2.destroyAllWindows()
    

def aprilTagDetection():
    coords = []
    img = cv2.imread("image copy.png")
    if img.dtype != "uint8":
        img = img.astype("uint8")
    
    img = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    
    detector = tag.AprilTagDetector()
    detector.addFamily('tag25h9')
    detector.addFamily('tag36h11')
    predictions = detector.detect(img)
    for pred in predictions:
        coords.append((pred.getCorner(3), pred.getCorner(1)))
        
    for coord in coords:
        cv2.rectangle(img, (int(coord[0].x), int(coord[0].y)), (int(coord[1].x), int(coord[1].y)), color=(34,34,34), thickness=2)
    
    cv2.imshow('frame', cv2.cvtColor(img, cv2.COLOR_BGR2RGB))
    

def poseEstimation():
    img = cv2.imread("image.png")
    if img.dtype != "uint8":
        img = img.astype("uint8")
    
    img = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    

    

open_camera()
    