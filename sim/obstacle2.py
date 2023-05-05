from __future__ import print_function
import cv2 as cv


class Obstacle:
    def __init__(self, tem, num_tem) -> None:
        self.num_tem = num_tem
        self.tem = tem
        self.match_method = 5
        self.maxVal = [0 for i in range(self.num_tem)]

    def detect_obstacle(self, img):
        result = []
        for i in range(self.num_tem):
            result.append(None)

        result[0] = cv.matchTemplate(img, self.tem[0], self.match_method)
        _, self.maxVal[0], _, self.maxLoc = cv.minMaxLoc(result[0], None)
        print('Score of template n.0 = ' + str(self.maxVal[0]))
        if (self.maxVal[0] >= 0.60):
            self.maxVal[0] += 1
        elif (self.maxVal[0] < 0.55):
            self.maxVal[0] = 0
        car_score = self.maxVal[0]
        rb_score = 0

        for i in range(1, self.num_tem, 1):
            result[i] = cv.matchTemplate(img, self.tem[i], self.match_method)
            _, self.maxVal[i], _, self.maxLoc = cv.minMaxLoc(result[i], None)
            print('Score of template n.' + str(i) + ' = ' +
                  str(self.maxVal[i]))
            if (self.maxVal[i] >= 0.65):
                self.maxVal[i] += 1
            elif (self.maxVal[i] < 0.6):
                self.maxVal[i] = 0
            if (i < 5):
                car_score += self.maxVal[i]
            # else:
            #     rb_score += self.maxVal[i]

            print('Total car score: ' + str(car_score))
            # print('Total roadblock score: ' + str(rb_score))

        if car_score > 4 or rb_score > 4:
            if (car_score >= rb_score):
                return 'car'
            else:
                return 'roadblock'
        else:
            return 'pedestrian'
