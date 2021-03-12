import numpy as np
import cv2
import time
import sys

THRESHOLD = 5
MOVE_RATE = 0.5
timeout = 10

# Shi-Tomasi corner detection parameter for Feature points
st_params = {   "maxCorners":100,
                "qualityLevel":0.3,
                "minDistance":7,
                "blockSize":7
            }

# Lucas-Kanade method parameters
lk_params = {   "winSize":(15,15),
                "maxLevel": 2,
                "criteria":(cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_COUNT, 10, 0.03)
            }

# create random 100 color for plotting
color = np.random.randint(0, 255, (100, 3))

# compute euclidean distance
def euclidean_distance(x1,y1,x2,y2):
    v1 = np.array([x1,y1])
    v2 = np.array([x2,y2])
    return np.linalg.norm(v1-v2)

# calculating avarage optical flow velocity of the video
def avarage_velocity(good_prev,good_next):
    vs = []
    for next_point, prev_point in zip(good_next, good_prev):
            # get x-y coordinates
            prev_x, prev_y = prev_point.ravel()
            next_x, next_y = next_point.ravel()
            distance = euclidean_distance(prev_x, prev_y, next_x, next_y)
            vs.append(distance)
    avg_vel = sum(vs) / len(vs)
    return avg_vel

# calculating sum of optical flow velocity of the video
def sum_velocity(good_prev,good_next):
    vs = []
    for next_point, prev_point in zip(good_next, good_prev):
            # get x-y coordinates
            prev_x, prev_y = prev_point.ravel()
            next_x, next_y = next_point.ravel()
            distance = euclidean_distance(prev_x, prev_y, next_x, next_y)
            vs.append(distance)
    avg_vel = sum(vs)
    return avg_vel

# judge whether the object is moving or not
def optical_flow_detect(video=None,show=True):
    moved = []
    # prepare with first frame 
    end_flag, frame = cap.read()
    gray_prev = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    # get features
    feature_prev = cv2.goodFeaturesToTrack(gray_prev, mask = None, **st_params)
    flow_mask = np.zeros_like(frame)

    # store avarage velocities for optical flow
    avg_velos = []
    temp_velos = []
    step = 0
    start = time.time()

    while end_flag:
        # convert to gray scale
        gray_next = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

        # detect optical flow
        feature_next, status, err = cv2.calcOpticalFlowPyrLK(gray_prev, gray_next, feature_prev, None, **lk_params)

        # chose feature points which detect optical flow（0：not detected、1：detected）
        good_prev = feature_prev[status == 1]
        good_next = feature_next[status == 1]

        # calculate avarage velocity of features during 10 steps
        # either use sum of velocity or avarage of velocity
        velocity = avarage_velocity(good_prev,good_next)
        #velocity = sum_velocity(good_prev,good_next)
        avg_velos.append(velocity)
        temp_velos.append(velocity)
        if step % 10 == 0:
            total_velocity = sum(temp_velos)
            if total_velocity > THRESHOLD:
                moved.append(1)
                #print("Something Moving in the video")
            else:
                moved.append(0)
                #print("Nothing Moving in the video")
            temp_velos = []
        else:
            pass

        if show:
            # plot optical flow
            for i, (next_point, prev_point) in enumerate(zip(good_next, good_prev)):
                # get x-y coordinate for plotting
                prev_x, prev_y = prev_point.ravel()
                next_x, next_y = next_point.ravel()
                # plot optical flow lint to "flow_mask"
                flow_mask = cv2.line(flow_mask, (next_x, next_y), (prev_x, prev_y), color[i].tolist(), 2)
                # plot current point of features
                frame = cv2.circle(frame, (next_x, next_y), 5, color[i].tolist(), -1)

            # add images with points and optical flow lines
            img = cv2.add(frame, flow_mask)
            # show
            cv2.imshow('window', img)
        else:
            pass

        # quit with ESC
        if cv2.waitKey(30) & 0xff == 27:
            break

        # quit with timeout
        if time.time() - start > timeout:
            break

        # prepare for next frame
        gray_prev = gray_next.copy()
        feature_prev = good_next.reshape(-1, 1, 2)
        end_flag, frame = cap.read()
        step += 1
    
    # calculate the moving rate in the video
    move_rate = sum(moved) /len(moved)
    move = True if move_rate > MOVE_RATE else False
    return move, avg_velos

if __name__ == '__main__':
    if sys.argv[1]=="camera":
        # use camera
        cap = cv2.VideoCapture(0)
    else:
        # get video object
        cap = cv2.VideoCapture(sys.argv[1])
    # run optical flow detecter
    moved, avgVels = optical_flow_detect(video=cap)
    print("Moving detected ? ",moved)
    # finish
    cv2.destroyAllWindows()
    cap.release()