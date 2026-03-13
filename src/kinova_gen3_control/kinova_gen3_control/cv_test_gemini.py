import cv2
import numpy as np
import os

def run_line_fit_tracker():
    ip_address = "192.168.1.10"
    rtsp_url = f"rtsp://admin:admin@{ip_address}:554/color"
    
    # UDP is better for live tracking to avoid frame backup
    os.environ['OPENCV_FFMPEG_CAPTURE_OPTIONS'] = 'rtsp_transport;udp'
    
    cap = cv2.VideoCapture(rtsp_url)
    points_buffer = []

    if not cap.isOpened():
        print("Error: Could not connect to camera.")
        return

    print("Tracking... move a red object to see the line of best fit.")

    while True:
        ret, frame = cap.read()
        if not ret:
            break

        # 1. Color Masking (HSV)
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv, np.array([0, 120, 70]), np.array([10, 255, 255])) + \
               cv2.inRange(hsv, np.array([170, 120, 70]), np.array([180, 255, 255]))

        # 2. Find Contours and Centroid
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        if contours:
            largest_cnt = max(contours, key=cv2.contourArea)
            if cv2.contourArea(largest_cnt) > 500:
                M = cv2.moments(largest_cnt)
                if M["m00"] != 0:
                    cX = int(M["m10"] / M["m00"])
                    cY = int(M["m01"] / M["m00"])
                    
                    points_buffer.append((cX, cY))
                    if len(points_buffer) > 10:
                        points_buffer.pop(0)

                    # Draw current location
                    cv2.circle(frame, (cX, cY), 5, (0, 255, 0), -1)

        # 3. Calculate Line of Best Fit
        if len(points_buffer) == 10:
            pts = np.array(points_buffer, dtype=np.int32)
            
            # fitLine returns a vector (vx, vy) and a point (x0, y0)
            res = cv2.fitLine(pts, cv2.DIST_L2, 0, 0.01, 0.01)
            
            # EXTRACT DATA SAFELY (Fixed for NumPy 1.25+)
            vx = float(res[0].item())
            vy = float(res[1].item())
            x0 = float(res[2].item())
            y0 = float(res[3].item())

            # 4. Draw Line and Calculate Intercept
            if vx != 0:  # Avoid division by zero for vertical lines
                m = vy / vx
                b = y0 - (m * x0)
                
                cols = frame.shape[1]
                
                # Find Y at screen edges
                y_left = b
                y_right = (m * cols) + b

                # SAFETY: Only draw if the numbers are valid (not NaN or Inf)
                if np.isfinite(y_left) and np.isfinite(y_right):
                    # Draw blue trajectory line
                    cv2.line(frame, (cols - 1, int(y_right)), (0, int(y_left)), (255, 0, 0), 2)
                    
                                # 5. Find X-Intercept at the BOTTOM of the frame (y = h)
                    if m != 0:
                        # Get the actual height of the frame
                        h = frame.shape[0] 
                        
                        # Equation: y = mx + b  =>  h = mx + b  => x = (h - b) / m
                        intercept_x = (h - b) / m
                        
                        if np.isfinite(intercept_x):
                            # Draw the BIG BLUE POINT at the bottom intercept
                            # Coordinates are (x, h-1) to keep it visible on the last pixel row
                            cv2.circle(frame, (int(intercept_x), h - 1), 25, (255, 0, 0), -1)
                            
                            # Update text to show it's targeting the bottom
                            cv2.putText(frame, "TARGET", (int(intercept_x) - 40, h - 40), 
                                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 0, 0), 2)

        cv2.imshow("Kinova Line Fitting", frame)

        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    cap.release()
    cv2.destroyAllWindows()

if __name__ == "__main__":
    run_line_fit_tracker()