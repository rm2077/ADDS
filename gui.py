import customtkinter as ctk
import tkinter as tk
from tkinter import messagebox
import sqlite3
import bcrypt
from PIL import Image, ImageTk
import cv2
from paho.mqtt import client as mqtt_client
import logging
import time
import socket

client_id = 'python-mqtt-600'
broker = 'broker.hivemq.com'
port = 1883
topic = 'drone/new'

FIRST_RECONNECT_DELAY = 1
RECONNECT_RATE = 2
MAX_RECONNECT_COUNT = 12
MAX_RECONNECT_DELAY = 60
# Database setup
def create_db():
    conn = sqlite3.connect('users.db')
    c = conn.cursor()
    c.execute('''CREATE TABLE IF NOT EXISTS users
                 (username TEXT PRIMARY KEY, password TEXT)''')
    conn.commit()
    conn.close()

create_db()

class LoginWindow(ctk.CTk):
    def __init__(self, on_login_success):
        super().__init__()
        self.on_login_success = on_login_success
        self.title("Login")
        self.geometry("300x400")

        self.create_widgets()

    def create_widgets(self):
        # Load the image
        self.image = Image.open("SkySwiftIMG.png")
        self.image = self.image.resize((100, 100), Image.BILINEAR)  # Resize the image
        self.photo = ImageTk.PhotoImage(self.image)

        # Create an image label
        self.image_label = ctk.CTkLabel(self, image=self.photo)
        self.image_label.pack(pady=10)

        # Username
        self.username_entry = ctk.CTkEntry(self, placeholder_text="Username")
        self.username_entry.pack(pady=12, padx=20)

        # Password
        self.password_entry = ctk.CTkEntry(self, placeholder_text="Password", show="*")
        self.password_entry.pack(pady=12, padx=20)

        # Login Button
        self.login_button = ctk.CTkButton(self, text="Login", command=self.login)
        self.login_button.pack(pady=10)

        # Register Button (below the Login Button)
        self.switch_to_register_button = ctk.CTkButton(self, text="Register", command=self.show_register)
        self.switch_to_register_button.pack(pady=10)

    def login(self):
        username = self.username_entry.get()
        password = self.password_entry.get()

        conn = sqlite3.connect('users.db')
        c = conn.cursor()
        c.execute('''SELECT password FROM users WHERE username=?''', (username,))
        stored_password = c.fetchone()
        conn.close()

        if stored_password and bcrypt.checkpw(password.encode('utf-8'), stored_password[0]):
            self.destroy()  # Close the login window
            self.on_login_success()  # Open the main UI
        else:
            messagebox.showerror("Error", "Invalid credentials")

    def show_register(self):
        self.withdraw()  # Hide the login window
        RegisterWindow(self).mainloop()  # Open the registration window

class RegisterWindow(ctk.CTk):
    def __init__(self, login_window):
        #Open camera
        super().__init__()
        self.login_window = login_window
        self.title("Register")
        self.geometry("300x300")
        #self.init_camera(cap)
        self.create_widgets()

    #30 Frames stored in array, loaded into init camera function in GUI sequentially
    #arr.append(frame)
    def init_camera(self, video):
        while video.isOpened():
            ret, frame = video.read()
            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)


    def create_widgets(self):
        # Username
        self.username_entry = ctk.CTkEntry(self, placeholder_text="Username")
        self.username_entry.pack(pady=12, padx=20)

        # Password
        self.password_entry = ctk.CTkEntry(self, placeholder_text="Password", show="*")
        self.password_entry.pack(pady=12, padx=20)

        # Register Button
        self.register_button = ctk.CTkButton(self, text="Register", command=self.register)
        self.register_button.pack(pady=20)

        # Back to Login Button
        self.back_button = ctk.CTkButton(self, text="Back to Login", command=self.back_to_login)
        self.back_button.pack(pady=10)

    def register(self):
        username = self.username_entry.get()
        password = self.password_entry.get()

        if not username or not password:
            messagebox.showerror("Error", "Username and password are required")
            return

        # Hash the password
        hashed_password = bcrypt.hashpw(password.encode('utf-8'), bcrypt.gensalt())

        conn = sqlite3.connect('users.db')
        c = conn.cursor()
        try:
            c.execute('''INSERT INTO users (username, password) VALUES (?, ?)''', (username, hashed_password))
            conn.commit()
            messagebox.showinfo("Success", "Registration successful!")
            self.back_to_login()
        except sqlite3.IntegrityError:
            messagebox.showerror("Error", "Username already exists")
        finally:
            conn.close()

    def back_to_login(self):
        self.destroy()
        self.login_window.deiconify()  # Show the login window again
class MainApp(ctk.CTk):
    def __init__(self, frame, data):
        super().__init__()
        self.frame = frame
        self.data = data
        self.title("SkySwift Deliveries")
        self.geometry("1000x1000")
        self.configure(bg='#2E2E2E')
        #self.update_camera(self.frame)
        #self.update_data(self.data)
        self.create_widgets()
        #self.cap = cap
        #ret, frame = self.cap.read()
        #self.create_widgets(frame)
    
        #Drone X, Drone Y, Drone Z, Pitch, Yaw, Roll
            
    

    def create_widgets(self):
        self.title_label = ctk.CTkLabel(master=self, text="Sky Swift Deliveries", font=("Arial", 20, "bold"))
        self.title_label.pack(pady=10)

        self.label1 = ctk.CTkLabel(master=self, text="Delivery ID:", font=("Roboto", 14))
        self.label1.pack(pady=10)

        self.delivery_id_entry = ctk.CTkEntry(master=self, placeholder_text="Enter Delivery ID")
        self.delivery_id_entry.pack(pady=5, padx=10, fill=tk.X)

        self.delivery_location_label = ctk.CTkLabel(master=self, text="Delivery Location: ", font=("Roboto", 14))
        self.delivery_location_label.pack(pady=5, padx=10, fill=tk.X)

        self.delivery_location_entry = ctk.CTkEntry(master=self, placeholder_text="Enter Delivery Location: ")
        self.delivery_location_entry.pack(pady=5, padx=10, fill=tk.X)
       

        self.submit_button = ctk.CTkButton(master=self, text="Deliver", command=self.update_status)
        self.submit_button.pack(pady=10)

        self.camera_label = ctk.CTkLabel(master=self, text="Drone Camera Stream", font=("Roboto", 14))
        self.camera_label.pack(pady=10)

        self.droneCameraFrame = ctk.CTkFrame(master=self, width=500, height=500)
        self.droneCameraFrame.pack(pady=10)

        self.stream_label = ctk.CTkLabel(master=self, text="")
        self.stream_label.pack(pady=1)

        self.status_label = ctk.CTkLabel(master=self, text="Status: ", font=('Roboto', 16, 'bold'))
        self.status_label.pack(pady=10)

        self.eta_label = ctk.CTkLabel(master=self, text="ETA: ", font=('Roboto', 14))
        self.eta_label.pack(pady=5)

        self.coordinates_label = ctk.CTkLabel(master=self, text="Drone Position (X, Y, Z)", font=('Roboto', 14))
        self.coordinates_label.pack(pady=5)

        self.flight_dynamics_label = ctk.CTkLabel(master=self, text="Drone Flight Dynamics (Roll, Pitch, Yaw): ", font=("Roboto", 14))
        self.flight_dynamics_label.pack(pady=5)

        




    def update_camera(self, frame):
        im = Image.fromarray(frame)
        Imgtk = ctk.CTkImage(im, size=(300, 300))
        self.stream_label.imgtk = Imgtk
        self.stream_label.configure(image=Imgtk)

        #self.after(500, self.update_camera)
        
    def update_data(self, data):
        self.data_label.configure(text=data)

        #self.after(500, self.update_data)

    #####################################################################################################################


    def update_status(self):
        delivery_id = self.delivery_id_entry.get()
        status = self.status_entry.get()
        messagebox.showinfo("Info", f"Delivery ID: {delivery_id}, Status: {status}")

#####################################################################################################################
    def new_button_clicked(self):
        messagebox.showinfo("Info", "New button clicked")

#####################################################################################################################


def connect_mqtt():
    def on_connect(client, userdata, flags, rc, properties):
        if rc == 0:
            print("Connected to MQTT Broker!")
        else:
            print("Failed to connect, return code %d\n", rc)

    def on_disconnect(client, userdata, rc):
        logging.info("Disconnected with result code: %s", rc)
        reconnect_count, reconnect_delay = 0, FIRST_RECONNECT_DELAY
        while reconnect_count < MAX_RECONNECT_COUNT:
            logging.info("Reconnecting in %d seconds...", reconnect_delay)
            time.sleep(reconnect_delay)

            try:
                client.reconnect()
                logging.info("Reconnected successfully!")
                return
            except Exception as err:
                logging.error("%s. Reconnect failed. Retrying...", err)

            reconnect_delay *= RECONNECT_RATE
            reconnect_delay = min(reconnect_delay, MAX_RECONNECT_DELAY)
            reconnect_count += 1
        logging.info("Reconnect failed after %s attempts. Exiting...", reconnect_count)

    client = mqtt_client.Client(client_id=client_id, callback_api_version=mqtt_client.CallbackAPIVersion.VERSION2)
    client.on_connect = on_connect
    client.connect(broker, port)
    return client



def subscribe(client: mqtt_client):
    def on_message(client, userdata, msg):
        print(f"Received `{msg.payload.decode()}` from `{msg.topic}` topic")

    client.subscribe(topic)
    time.sleep(1)
    client.on_message = on_message



def run_mqtt():
    client = connect_mqtt()
    subscribe(client)
    client.loop_forever()
    

def start_main_app(frame=None, data=None):
            main_app = MainApp(frame, data)
            main_app.mainloop()
           

#login_window = LoginWindow(on_login_success=start_main_app)
#login_window.mainloop()

if __name__ == "__main__":
    run_mqtt()
    












