## 1.	Mediapipe, Hands定义失败
教程上是
```
self.hands = self.mpHands.Hands(self.mode, self.maxHands,
                                self.detectionCon, self.trackCon)
```
但由于Mediapipe,升级，变成了五个参数，因为以为要养成点击看方法的习惯
```
self.hands = self.mpHands.Hands(self.mode, self.maxHands,
                                self.modelCom, self.detectionCon, self.trackCon)
```
## 2.	获取摄像头的方法，加上cv2.CAP_DSHOW
```cap = cv2.VideoCapture(0 + cv2.CAP_DSHOW)```
