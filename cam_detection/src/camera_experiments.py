# import required libraries
import cv2
import numpy as np


def img_contours(img_to_contour, to_draw):

    contours, hierarchy = cv2.findContours(image=img_to_contour, mode=cv2.RETR_TREE, method=cv2.CHAIN_APPROX_NONE)

    mask = np.zeros(img_to_contour.shape, np.uint8)

    boxes = []

    mean_clr_lst = []

    for i in range(0, len(contours)):
        mask[...] = 0
        cv2.drawContours(mask, contours, i, (255, 0, 0), 10)
        mean_clr = cv2.mean(to_draw, mask)
        mean_clr_lst = [round(mean_clr[0]), round(mean_clr[1]), round(mean_clr[2])]

        (x, y, w, h) = cv2.boundingRect(contours[i])
        boxes.append([x, y, x + w, y + h])

    return boxes, mean_clr_lst


def drawing_over(img, boxes, txt_var):
    boxes = np.asarray(boxes)
    left, top = np.min(boxes, axis=0)[:2]
    right, bottom = np.max(boxes, axis=0)[2:]

    cv2.rectangle(img, (left, top), (right, bottom), (255, 0, 0), 2)
    cv2.putText(img, txt_var, (left, bottom), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 0, 0))

    return img


def img_edges(img_to_edge):
    to_draw = cv2.Canny(img_to_edge, 70, 135)
    return to_draw


def img_blur(img_to_blur):
    ret_img = cv2.medianBlur(img_to_blur, 29)
    ret_img = cv2.blur(ret_img, (10, 10))

    return ret_img


def img_get(cap_to_get_from):
    ret_to_ret, frame_to_ret = cap_to_get_from.read()

    return ret_to_ret, frame_to_ret


def img_show(img_to_show):
    cv2.imshow('frame', img_to_show)


def img_thresh(img_to_thresh):
    img_gray = cv2.cvtColor(img_to_thresh, cv2.COLOR_RGB2GRAY)
    ret, thresh = cv2.threshold(img_gray, 50, 200, cv2.THRESH_BINARY)

    thresh = cv2.bitwise_not(thresh)
    # thresh = cv2.cvtColor(thresh, cv2.COLOR_BGR2GRAY)
    return thresh


def main():
    cap = cv2.VideoCapture(1)

    while True:
        ret, frame = img_get(cap)

        if not ret:
            print("Can't receive frame (stream end?). Exiting ...")
            break

        blurred = img_blur(frame)
        # contoured = img_contours(blurred)
        threshed = img_thresh(blurred)
        edged = img_edges(threshed)
        boxes_array, mean_clrs = img_contours(edged, frame)
        final = frame

        if mean_clrs != []:
            txt_var = "Eggplant"
            if mean_clrs[2] > 85:
                txt_var = "Tomato"
            final = drawing_over(frame, boxes_array, txt_var)
        img_show(final)
        # img_show(mean_clr)

        if cv2.waitKey(1) == ord('q'):
            break


main()
