import cv2

cam = cv2.VideoCapture(0)
cv2.namedWindow("Image")

qrCodeDetector = cv2.QRCodeDetector()

while True:
    _,image = cam.read()

    # cv2.imshow("Qrcode detector", image)

    decodedText, points, _ = qrCodeDetector.detectAndDecode(image)
    if points is not None:

        nrOfPoints = len(points)

        for i in range(nrOfPoints):
            nextPointIndex = (i+1) % nrOfPoints
            cv2.line(image, tuple(points[i][0]), tuple(points[nextPointIndex][0]), (255,0,0), 5)

        print(decodedText)

        cv2.imshow("Image", image)


    else:
        print("QR code not detected")

    cv2.waitKey(1)
