from camera import Camera

import cv2

class StereoMatch:

    def __init__(self):
        self.c = Camera()

    def main(self):
        processing_time = []
        while True:
            processing_time01 = cv2.getTickCount()
            imgLeft, disparity = self.c.create_disparity_map()
            cv2.imshow('image',img)
            processing_time.append(cv2.getTickCount() - processing_time01)
            K = cv2.waitKey(0)
            if k ==27:
                cv2.destroyAllWindows()
                print("Median processing time:", sorted(processing_time)[int(len(processing_time)/2)])


if __name__ == '__main__':
    main()
