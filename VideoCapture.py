import numpy as np
import cv2
import grasp_image_func as gi

cap = cv2.VideoCapture(0)
while(True):
    # Capture frame-by-frame
    ret, frame = cap.read()
    # img = cv2.copyMakeBorder(frame, 0, 0, 0, 0, cv2.BORDER_REPLICATE)
    # cv2.imshow('img', img)
    # cv2.waitKey()
    # Our operations on the frame come here
    # gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    # draw grasps
    if frame is None:
        continue
    else:
        H, W, Ang = gi.predictGraspOnImage(frame)
        print(H)  # y-coord of pixel position
        print(W)  # x-coord of pixel position
        print(Ang)  # angle of grasp
    # Display the resulting frame
    # cv2.imshow('frame', frame)

    usrInput = raw_input('Press enter to continue or "q" to quit')

    if usrInput == 'q':
        print('exiting...')
        break
    else:
        print('return to loop')
        print('----------->')

    # if cv2.waitKey(1) & 0xFF == ord('q'):
    #     break

# try:
#     while True:
#         ret, frame = cap.read()
#         if frame is None:
#             continue
#         else:
#             gi.predictGraspOnImage(frame)
# except KeyboardInterrupt:
#     pass

# When everything done, release the capture
cap.release()
cv2.destroyAllWindows()

