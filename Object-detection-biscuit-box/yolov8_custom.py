# # # from ultralytics import YOLO
# # # import cv2 as cv

# # # #load the model
# # # model = YOLO("best.pt") # load a pretrained model

# # # # use the model
# # # results = model(source = 0, show = True, conf=0.9, save=True)

# # # for result in results:
# # #     # boxes = result.boxes.xyxy  # Get box coordinates in (top, left, bottom, right) format
# # #     # conf = result.boxes.conf  # Confidence scores
# # #     # print(f"Object at ({boxes[0]}, {boxes[1]}), width={boxes[2] - boxes[0]}, height={boxes[3] - boxes[1]}")

# # #     x, y, w, h = result.bbox  # Get bounding box coordinates
# # #     print(f"Object at ({x}, {y}), width={w}, height={h}")



# # Code 2
# # from ultralytics import YOLO
# # import cv2

# # #load the model
# # model = YOLO('best.pt')

# # # capture video
# # cap = cv2.VideoCapture(0)
# # cap.set(3,640) #height
# # cap.set(6, 480) #width


# # while True:
# #     _, frame = cap.read()
# #     img = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)

# #     # Get YOLOv8 predictions
# #     results = model.predict(img)

# #     # Loop through each detected object
# #     for r in results:
# #         for box in r.boxes:
# #             # Get box coordinates in (left, top, right, bottom) format
# #             b = box.xyxy[0]
# #             # Get predicted class label
# #             c = box.cls
# #             print(f"Class: {model.names[int(c)]}, Coordinates: {b}")

# #     # Display the annotated frame
# #     cv2.imshow('YOLO V8 Detection', frame)

# #     # Press 'Space' to exit
# #     if cv2.waitKey(1) & 0xFF == ord(' '):
# #         break

# # cap.release()
# # cv2.destroyAllWindows()

# # Code 3
# from ultralytics import YOLO
# import cv2
# from ultralytics.utils.plotting import Annotator  # ultralytics.yolo.utils.plotting is deprecated

# model = YOLO('best.pt')
# cap = cv2.VideoCapture(0)
# cap.set(3, 640)
# cap.set(4, 480)

# while True:
#     _, img = cap.read()  # BGR to RGB conversion is performed under the hood (see: [^1^][6])
#     results = model.predict(img)

#     for r in results:
#         annotator = Annotator(img)
#         boxes = r.boxes
#         for box in boxes:
#             b = box.xyxy[0]  # Get box coordinates in (left, top, right, bottom) format
#             c = box.cls
#             annotator.box_label(b, model.names[int(c)])  # Annotate the box with class label

#     img = annotator.result()
#     cv2.imshow('YOLO V8 Detection', img)

#     if cv2.waitKey(1) & 0xFF == ord(' '):
#         break

# cap.release()
# cv2.destroyAllWindows()


# Code 4
from ultralytics import YOLO
import cv2
from ultralytics.utils.plotting import Annotator

model = YOLO('best.pt')
cap = cv2.VideoCapture(0)
cap.set(3, 640)
cap.set(4, 480)

while True:
    _, img = cap.read()
    results = model.predict(img)

    for r in results:
        annotator = Annotator(img)
        boxes = r.boxes
        for box in boxes:
            b = box.xyxy[0]  # Get box coordinates in (left, top, right, bottom) format
            c = box.cls
            annotator.box_label(b, model.names[int(c)])  # Annotate the box with class label

            # Display the coordinates on the image
            cv2.putText(img, f"({b[0]:.0f}, {b[1]:.0f})", (int(b[0]), int(b[1]) - 10),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1, cv2.LINE_AA)

    img = annotator.result()
    cv2.imshow('YOLO V8 Detection', img)

    if cv2.waitKey(1) & 0xFF == ord(' '):
        break

cap.release()
cv2.destroyAllWindows()
