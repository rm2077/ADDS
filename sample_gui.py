import cv2
import PySimpleGUI.PySimpleGUI as sg
from PIL import Image
import io
def displayVideoStream():
    cap = cv2.VideoCapture(1)
    while cap.isOpened():
        ret, frame = cap.read()
        if not ret:
            print("Camera unable to open...")
            break
        if cv2.waitKey(1) & 0xFF == ord("q"):
            break
        frame = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        img = Image.fromarray(frame)
        bio = io.BytesIO()
        img_bytes = bio.getvalue()
        return img_bytes
        
        

    cap.release()
    cv2.destroyAllWindows()


def main():
    layout = [[sg.Text('Frame', size=(400, 100), justification='center', font='Helvetica 20')],
              [sg.Image(filename='', key='image')],
              ]
     
    window = sg.Window('Drone Camera', location=(800,800))
    window.Layout(layout)
    cap = cv2.VideoCapture(1)
    
    while True:
        button, values = window._ReadNonBlocking()
        ret, frame = cap.read()
        if not ret:
            print("Camera unable to open...")
            break
        if cv2.waitKey(1) & 0xFF == ord("q"):
            break
        frame = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        img = Image.fromarray(frame)
        bio = io.BytesIO()
        img.save(bio, format= 'PNG')
        img_bytes = bio.getvalue()
        
        window.FindElement('image').Update(data=img_bytes)

main()