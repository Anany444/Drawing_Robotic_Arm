import cv2
import numpy as np

# ------------------ CONFIG ------------------
WIDTH, HEIGHT = 1000, 1000
BRUSH_RADIUS = 1
WINDOW_NAME = "Draw (Press S to save, Q to quit)"

coords_list = []
HEIGHT_TOOL = 0.2

# ------------------ GLOBAL STATE ------------------
drawing = False
canvas = np.ones((HEIGHT, WIDTH), dtype=np.uint8) * 255  # white background

# ------------------ MOUSE CALLBACK ------------------
def draw_callback(event, x, y, flags, param):
    global drawing, canvas

    if event == cv2.EVENT_LBUTTONDOWN:
        drawing = True
        cv2.circle(canvas, (x, y), BRUSH_RADIUS, 0, -1)
        if len(coords_list) == 0 or (abs(x-coords_list[-1][0])**2 + abs(y-coords_list[-1][1])**2 > 25):  # avoid duplicates
            coords_list.append([x , y])

    elif event == cv2.EVENT_MOUSEMOVE and drawing:
        cv2.circle(canvas, (x, y), BRUSH_RADIUS, 0, -1)
        if len(coords_list) == 0 or (abs(x-coords_list[-1][0])**2 + abs(y-coords_list[-1][1])**2 > 25):  # avoid duplicates
            coords_list.append([x , y])

    elif event == cv2.EVENT_LBUTTONUP:
        drawing = False
        cv2.circle(canvas, (x, y), BRUSH_RADIUS, 0, -1)

# ------------------ MAIN ------------------
def main():
    cv2.namedWindow(WINDOW_NAME)
    cv2.setMouseCallback(WINDOW_NAME, draw_callback)

    while True:
        cv2.imshow(WINDOW_NAME, canvas)
        key = cv2.waitKey(1) & 0xFF

        if key == ord('s'):
            cv2.imwrite("drawing.png", canvas)
            print("Saved drawing.png")

        elif key == ord('q') or key == 27:  # ESC
            break

    cv2.destroyAllWindows()
    for x in coords_list:
        x[0], x[1] = (f"{(x[0]-WIDTH/2)/3500:.4f}"), (f"{-0.25-x[1]/3500:.4f}")  # center coordinates
        x.append(HEIGHT_TOOL)  # add z coordinate
    file = open("coords.txt", "w")
    #print("Collected coordinates:", coords_list)
    output = "\n".join([f"{p[0]} {p[1]} {p[2]}" for p in coords_list])
    print(output)
    file.write(output)

    file.close()

if __name__ == "__main__":
    main()
