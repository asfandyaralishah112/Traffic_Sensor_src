import tkinter as tk
from tkinter import ttk, messagebox
import serial
import serial.tools.list_ports
import json
import os
import random
from PIL import Image, ImageTk
from openpyxl import Workbook, load_workbook
from datetime import datetime
import threading
import time

CONFIG_FILE = "provisioning_config.json"
EXCEL_FILE = "device_registry.xlsx"


class ProvisionTool:

    def __init__(self, root):
        self.root = root
        self.root.title("Cavline Global - Traffic Sensor Provisioning")
        self.root.geometry("560x650")
        self.root.configure(bg="#071218")

        self.create_ui()
        self.refresh_ports()
        self.load_config()

    # ==================================================
    # UI
    # ==================================================
    def create_ui(self):

        container = tk.Frame(self.root, bg="#071218")
        container.pack(fill="both", expand=True, padx=20, pady=15)

        # ---------- Logo ----------
        try:
            img = Image.open("logo.png")
            img = img.resize((260, int(260 * img.height / img.width)))
            self.logo_img = ImageTk.PhotoImage(img)

            logo_label = tk.Label(container,
                                  image=self.logo_img,
                                  bg="#071218")
            logo_label.pack(pady=(0, 10))
        except:
            pass

        title = tk.Label(container,
                         text="Traffic Sensor Gen 1 Provisioning",
                         bg="#071218",
                         fg="#e6eef3",
                         font=("Segoe UI", 14, "bold"))
        title.pack(pady=(0, 10))

        form = tk.Frame(container, bg="#071218")
        form.pack(fill="x")

        self.entries = {}

        fields = [
            ("Start UID", tk.Entry),
            ("COM Port", ttk.Combobox),
            ("MQTT Server", tk.Entry),
            ("MQTT Port", tk.Entry),
        ]

        for i, (label, widget) in enumerate(fields):

            tk.Label(form,
                     text=label,
                     bg="#071218",
                     fg="#e6eef3",
                     font=("Segoe UI", 10)).grid(
                         row=i, column=0, sticky="w", pady=6)

            if widget == ttk.Combobox:
                entry = widget(form, width=30, state="readonly")
            else:
                entry = widget(form, width=32)

            entry.grid(row=i, column=1, pady=6, padx=(10, 0))
            self.entries[label] = entry

        refresh_btn = ttk.Button(form,
                                 text="Refresh Ports",
                                 command=self.refresh_ports)
        refresh_btn.grid(row=1, column=2, padx=10)

        # ---------- Upload Button ----------
        self.upload_btn = tk.Button(container,
                                    text="UPLOAD TO DEVICE",
                                    command=self.start_upload_thread,
                                    bg="#1c7ed6",
                                    fg="white",
                                    font=("Segoe UI", 11, "bold"),
                                    relief="flat",
                                    height=2)
        self.upload_btn.pack(fill="x", pady=10)

        # ---------- Output Window ----------
        tk.Label(container,
                 text="Output",
                 bg="#071218",
                 fg="#8fa3ad").pack(anchor="w")

        self.output = tk.Text(container,
                              height=12,
                              bg="#0f171d",
                              fg="#e6eef3",
                              insertbackground="white")
        self.output.pack(fill="both", expand=True)

        # ---------- Progress Bar ----------
        self.progress = ttk.Progressbar(container,
                                        orient="horizontal",
                                        length=100,
                                        mode="determinate")
        self.progress.pack(fill="x", pady=10)

        footer = tk.Label(container,
                          text="Cavline Global © 2026",
                          bg="#071218",
                          fg="#7f949e",
                          font=("Segoe UI", 9))
        footer.pack(side="bottom")

    # ==================================================
    # OUTPUT LOGGING
    # ==================================================
    def log(self, text):
        self.output.insert(tk.END, text + "\n")
        self.output.see(tk.END)
        self.root.update_idletasks()

    # ==================================================
    # PROGRESS
    # ==================================================
    def set_progress(self, value):
        self.progress["value"] = value
        self.root.update_idletasks()

    # ==================================================
    # COM PORT DETECTION
    # ==================================================
    def refresh_ports(self):
        ports = serial.tools.list_ports.comports()
        port_list = [p.device for p in ports]

        combo = self.entries["COM Port"]
        combo["values"] = port_list

        if port_list:
            combo.current(0)

    # ==================================================
    # CONFIG LOAD/SAVE
    # ==================================================
    def load_config(self):
        if os.path.exists(CONFIG_FILE):
            with open(CONFIG_FILE, "r") as f:
                data = json.load(f)
                for k in self.entries:
                    if k in data:
                        self.entries[k].insert(0, data[k])

    def save_config(self):
        data = {k: self.entries[k].get() for k in self.entries}
        with open(CONFIG_FILE, "w") as f:
            json.dump(data, f, indent=4)

    # ==================================================
    # UID INCREMENT
    # ==================================================
    def increment_uid(self, uid):
        prefix, number = uid.rsplit("-", 1)
        new_number = str(int(number) + 1).zfill(6)
        return f"{prefix}-{new_number}"

    # ==================================================
    # PASSWORD GENERATION
    # ==================================================
    def generate_password(self, length=12):
        chars = "ABCDEFGHJKLMNPQRSTUVWXYZabcdefghijkmnopqrstuvwxyz23456789"
        return ''.join(random.choice(chars) for _ in range(length))

    # ==================================================
    # EXCEL LOGGING
    # ==================================================
    def log_to_excel(self, uid, user, password, topic, server):

        if not os.path.exists(EXCEL_FILE):
            wb = Workbook()
            ws = wb.active
            ws.append([
                "UID",
                "MQTT User",
                "MQTT Password",
                "MQTT Topic",
                "MQTT Server",
                "Date",
                "Time"
            ])
            wb.save(EXCEL_FILE)

        wb = load_workbook(EXCEL_FILE)
        ws = wb.active

        now = datetime.now()

        ws.append([
            uid,
            user,
            password,
            topic,
            server,
            now.strftime("%Y-%m-%d"),
            now.strftime("%H:%M:%S")
        ])

        wb.save(EXCEL_FILE)

    # ==================================================
    # THREAD START
    # ==================================================
    def start_upload_thread(self):
        t = threading.Thread(target=self.upload)
        t.start()

    # ==================================================
    # UPLOAD PROCESS
    # ==================================================
    def upload(self):

        self.progress["value"] = 0
        self.output.delete("1.0", tk.END)

        uid = self.entries["Start UID"].get()
        port = self.entries["COM Port"].get()

        if not uid or not port:
            messagebox.showerror("Error", "UID and COM Port required")
            return

        mqtt_user = uid
        mqtt_pass = self.generate_password()
        mqtt_topic = f"door/counter/events/{uid}"

        payload = {
            "cmd": "write_config",
            "uid": uid,
            "mqtt_server": self.entries["MQTT Server"].get(),
            "mqtt_port": int(self.entries["MQTT Port"].get()),
            "mqtt_user": mqtt_user,
            "mqtt_pass": mqtt_pass,
            "mqtt_topic": mqtt_topic
        }

        try:
            self.log("Opening serial port...")
            self.set_progress(20)

            ser = serial.Serial(port, 115200, timeout=3)
            time.sleep(1)

            self.log("Sending configuration...")
            self.set_progress(40)

            ser.write((json.dumps(payload) + "\n").encode())

            self.log("Waiting for ESP confirmation...")
            self.set_progress(60)

            response = ser.readline().decode().strip()
            ser.close()

            if response == "OK":

                self.log("Writing to Excel registry...")
                self.set_progress(80)

                self.log_to_excel(
                    uid,
                    mqtt_user,
                    mqtt_pass,
                    mqtt_topic,
                    self.entries["MQTT Server"].get()
                )

                new_uid = self.increment_uid(uid)
                self.entries["Start UID"].delete(0, tk.END)
                self.entries["Start UID"].insert(0, new_uid)

                self.save_config()

                self.set_progress(100)
                self.log("Provisioning completed successfully.")

                messagebox.showinfo("Success", f"Provisioned:\n{uid}")

            else:
                self.log("ERROR: ESP did not confirm")
                messagebox.showerror("Error", "ESP did not confirm")

        except Exception as e:
            self.log(f"ERROR: {str(e)}")
            messagebox.showerror("Error", str(e))


# ==================================================
# START APPLICATION
# ==================================================
root = tk.Tk()
app = ProvisionTool(root)
root.mainloop()
