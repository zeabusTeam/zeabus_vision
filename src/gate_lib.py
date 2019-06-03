from __future__ import division
import cv2


class Gate:
    ''' GATE Processing class.
    You can use this class to process ROBOSUB gate mission.
    Parameters:
        fileOrDevice (str,int): you can send this to OpenCV to open
    '''

    def __init__(self, fileToOpen=None):
        if fileToOpen is not None:
            self.device = cv2.VideoCapture(fileToOpen)
        else:
            self.device = None
        self.filename = fileToOpen
        self.last_detect = None

    def read(self):
        '''Read opedgesenned file and openImg Window
        '''

        while self.device.isOpened():
            retval, image = self.device.read()
            if not retval:
                break
            self.doProcess(image, True)
            key = cv2.waitKey(1)
            if key == ord('q'):
                break

    def doProcess(self, img, showImg=False):
        """Put image then get outputs

        Arguments:
            img {OpenCV Image} -- Input image

        Keyword Arguments:
            showImg {bool} -- Wanna Show Img for debugging ? (default: {False})

        Returns:
            list -- Found data. None or list of cx1,cy1,cx2,cy2,area
        """
        # small_image = cv2.resize(img, None, fx=0.25, fy=0.25)
        processed = self._process(img)
        if showImg:
            cv2.imshow(str(self.filename)+' ct', processed[4])
        if processed[5] is not None:
            diff = self.calcDiffPercent(processed[5], self.last_detect)
            cond = self.last_detect is None or diff[0] < 0.4
            if cond:
                self.last_detect = processed[5]
        return (processed[5], processed[2])

    def _process(self, img):
        def my_area(ct):
            x, y, w, h = cv2.boundingRect(ct)
            return w*h
        self.img_size = img.shape[0:2]
        k_size = int(self.img_size[0]/80)
        k_size += (k_size % 2)-1
        blured = cv2.medianBlur(img, k_size)
        gray = cv2.cvtColor(blured, cv2.COLOR_BGR2GRAY)
        clahe = cv2.createCLAHE(clipLimit=2.0, tileGridSize=(
            int(self.img_size[0]*0.15), int(self.img_size[0]*0.15)))
        gray = clahe.apply(gray)

        (_mu, sigma) = cv2.meanStdDev(gray)
        edges = cv2.Canny(img, _mu - 1.25*sigma, _mu + 1.25*sigma)

        slide_box_size = int(img_size[0]*0.02)
        noise_thresh = edges.sum()/edges.size
        if _mu > 127:
            for i in range(len(edges)-slide_box_size):
                interest_area = edges[i:i+slide_box_size]
                if interest_area.sum()/interest_area.size > noise_thresh*0.8:
                    interest_area.fill(0)

        _, cts, hi = cv2.findContours(edges, cv2.RETR_EXTERNAL,
                                      cv2.CHAIN_APPROX_SIMPLE)
        cts = sorted(cts, key=my_area, reverse=True)
        found = None
        withct = img.copy()
        for c in cts:
            x, y, w, h = cv2.boundingRect(c)
            c_area = cv2.contourArea(c)
            if (c_area/w/h > 0.4) or w*h/edges.size < 0.02:
                continue
            else:
                # withct = cv2.drawContours(withct, [c], 0, (0, 255, 255), 3)
                found = ((2*x+w)/img.shape[1]-1, (2*y+h)/img.shape[0] - 1,
                         2*x/img.shape[1]-1, 2*(x+w)/img.shape[1]-1,
                         c_area/w/h)
                diff = self.calcDiffPercent(found, self.last_detect)
                cond = self.last_detect is None or diff[0] < 0.3
                if cond:
                    cv2.rectangle(withct, (x, y), (x+w, y+h), (255, 255, 0), 3)
                    cv2.circle(withct, (int(x+w/2), int(y+h/2)),
                               4, (0, 255, 255), 4)
                    break
        return (gray, blured, edges, cts, withct, found)

    def calcDiffPercent(self, first, second):
        if first is None or second is None or len(first) < len(second):
            return [0]
        res = []
        for key, val in enumerate(first):
            res.append(abs(val-second[key])/2)
        return tuple(res)
