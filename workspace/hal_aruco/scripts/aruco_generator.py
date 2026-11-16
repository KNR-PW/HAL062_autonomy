import customtkinter as ctk
from tkinter import *
import numpy as np
import cv2 as cv
import os
from PIL import Image, ImageTk

##################################################################
##   HOW TO USE                                                 ##
##   1. Install all libs                                        ##
##   2. enter parameters in gui                                 ##
##   3. click 'CONFIGURE' so your parameters are saved,         ##
##       the folder to save tags MUST EXIST BEFORE using app    ##
##   4. click 'CREATE'                                          ##
##   5. remebmer to change name of png in gui                   ##
##   6. repeat                                                  ##
##################################################################



USER = os.getlogin()     # your username
CONVERT = 37.795275591   # 1cm = CONVERT pixels
X = int(21 * CONVERT)    # A4 x size in pixels
Y = int(29.7 * CONVERT)  # A4 y size in pixels

TAG = None              # for tag view
TAG_VIEW = None         # for tag view 


SIDE = None                 # GUI
ID = None                   # GUI
BORDER = None               # GUI
TYPE = None                 # GUI
PNG_NAME = None             # GUI
PNG_PATH = None             # GUI




"""------------ARUCO-LAYER------------"""
ARUCO_DICT = {
                "DICT_4X4_50": cv.aruco.DICT_4X4_50,
                "DICT_4X4_100": cv.aruco.DICT_4X4_100,
                "DICT_4X4_250": cv.aruco.DICT_4X4_250,
                "DICT_4X4_1000": cv.aruco.DICT_4X4_1000,
                "DICT_5X5_50": cv.aruco.DICT_5X5_50,
                "DICT_5X5_100": cv.aruco.DICT_5X5_100,
                "DICT_5X5_250": cv.aruco.DICT_5X5_250,
                "DICT_5X5_1000": cv.aruco.DICT_5X5_1000,
                "DICT_6X6_50": cv.aruco.DICT_6X6_50,
                "DICT_6X6_100": cv.aruco.DICT_6X6_100,
                "DICT_6X6_250": cv.aruco.DICT_6X6_250,
                "DICT_6X6_1000": cv.aruco.DICT_6X6_1000,
                "DICT_7X7_50": cv.aruco.DICT_7X7_50,
                "DICT_7X7_100": cv.aruco.DICT_7X7_100,
                "DICT_7X7_250": cv.aruco.DICT_7X7_250,
                "DICT_7X7_1000": cv.aruco.DICT_7X7_1000
             }

def create_board() -> np.array:
    """creates a whiteboard of A4 size"""
    x_size: int = X       
    y_size: int = Y       
    board : np.array = np.zeros((y_size,x_size,3), dtype=np.uint8)
    board[:,:,:] = 255
    for x in range(100,X-100):
        for y in range(100,111):
            board[y,x,:] = 0
            board[y+100,x,:] = 0
    for x in range(100,211):
        for y in range(100,111):
            board[x,y,:] = 0
            board[x,X-200+y,:] = 0
    return board


def create_marker() -> np.array:
    """creates an ArUco marker of given size"""
    side_px: int = int(SIDE * CONVERT)
    dictionary: cv.aruco.Dictionary = cv.aruco.getPredefinedDictionary(ARUCO_DICT[TYPE])
    marker_bin: np.array = np.zeros((side_px,side_px,1), dtype=np.uint8)
    cv.aruco.generateImageMarker(dictionary=dictionary, 
                                 id=ID, 
                                 sidePixels=side_px,
                                 img=marker_bin,
                                 borderBits=BORDER)
    marker_rgb: np.array = np.zeros((side_px,side_px,3), dtype=np.uint8)
    for x in range(0,side_px):
        for y in range(0,side_px):
            if marker_bin[x,y] == [0]:
                marker_rgb[x,y,0] = 0
                marker_rgb[x,y,1] = 0
                marker_rgb[x,y,2] = 0
            elif marker_bin[x,y] == [255]:
                marker_rgb[x,y,0] = 255
                marker_rgb[x,y,1] = 255
                marker_rgb[x,y,2] = 255
    return marker_rgb


def create_tag(board: np.array, marker: np.array) -> None:
    """connects marker with white A4-sized board"""
    global TAG, TAG_VIEW
    distance: int = int((X - marker.shape[0])/2)
    tag = board[:]
    tag[tag.shape[0] - distance - marker.shape[0]:tag.shape[0] - distance,distance:distance+marker.shape[0]] = marker

    text: str = str(ID)
    font = cv.FONT_HERSHEY_SCRIPT_SIMPLEX
    textsize: tuple = cv.getTextSize(text, font, 1, 2)[0]
    textX = int((tag.shape[1] - textsize[0]) / 2)
    TAG = cv.putText(img=tag, text=text, org=(textX, 190), thickness=10, color=(0,0,0),
                     fontFace=font, fontScale=3, lineType=cv.LINE_AA)
    

def create_ar_tag(frame) -> None:
    """connects every element mentioned before"""
    board = create_board()
    marker = create_marker()
    create_tag(board,marker)

    path: os.PathLike = os.path.join(str(PNG_PATH),str(PNG_NAME))
    cv.imwrite(path,TAG)
    
    img = Image.open(path)
    TAG_VIEW = ctk.CTkImage(light_image=img, dark_image=img, size=(300,424))
    image_label = ctk.CTkLabel(master=frame, height=424, width=300, text="", fg_color="gray", corner_radius=5, image=TAG_VIEW)
    image_label.place(relx=0.6, rely=0.1)



"""------------GUI-LAYER------------"""
def create_root() -> ctk.CTk:
    """creates customtkinter gui"""
    global TAG_VIEW, FRAME
    root = ctk.CTk()
    ctk.set_appearance_mode("system")
    ctk.set_default_color_theme("green")
    root.geometry("1000x600")
    root.title("ArUco Tag Creator")

    frame = ctk.CTkFrame(master=root)
    frame.pack(pady=20,padx=60,fill="both",expand=True)


    image_label = ctk.CTkLabel(master=frame, height=424, width=300, text="Created ArTag", fg_color="gray", corner_radius=5)
    image_label.place(relx=0.6, rely=0.1)


    create_button = ctk.CTkButton(master=frame, width=450, height=60,
                        text_color="black", border_width=0, text="CREATE", command=lambda : create_ar_tag(frame),
                        font=("Helvetica",24, "bold"), corner_radius=10)
    create_button.place(relx=0.05, rely=0.85)


    configure_button = ctk.CTkButton(master=frame, width=450, height=60,
                        text_color="black", border_width=0, text="CONFIGURE", fg_color="orange",
                        command=lambda : set_variables(path_var, png_var, id_var, side_var, border_var, dict_var), 
                        font=("Helvetica",24, "bold"),corner_radius=10)
    configure_button.place(relx=0.05, rely=0.7)


    path_var = ctk.StringVar(value=f"/home/{USER}/Desktop")
    path_entry = ctk.CTkEntry(master=frame, height=15, width=300, text_color="white",
                             placeholder_text="Path to Folder", textvariable=path_var)
    path_entry.place(relx=0.2, rely=0.1)
    path_label = ctk.CTkLabel(master=frame, height=10, width=100, text="folder path", fg_color="gray", corner_radius=5)
    path_label.place(relx=0.05, rely=0.105)


    png_var = ctk.StringVar(value="test.png")
    png_entry = ctk.CTkEntry(master=frame, height=15, width=300, text_color="white",
                             placeholder_text="Name of PNG", textvariable=png_var)
    png_entry.place(relx=0.2,rely=0.2)
    png_label = ctk.CTkLabel(master=frame, height=10, width=100, text="file name", fg_color="gray",corner_radius=5)
    png_label.place(relx=0.05, rely=0.205)
    remind_label = ctk.CTkLabel(master=frame, height=10, width=100, text="Remember to change filename after each 'CREATE'", text_color="red")
    remind_label.place(relx=0.21,rely=0.17)


    dict_var = ctk.StringVar(value="DICT_5X5_50")
    dict_option = ctk.CTkOptionMenu(master=frame, height=15, width=300, values=list(ARUCO_DICT.keys()),
                                    text_color="white",dropdown_text_color="white", variable=dict_var)
    dict_option.place(relx=0.2,rely=0.301)
    dict_label = ctk.CTkLabel(master=frame, height=10, width=100, text="dictionary", fg_color="gray", corner_radius=5)
    dict_label.place(relx=0.05, rely=0.305)


    id_var = ctk.IntVar(value=0)
    id_entry = ctk.CTkEntry(master=frame, height=15, width=300, text_color="white",placeholder_text="ID", textvariable=id_var)
    id_entry.place(relx=0.2,rely=0.4)
    id_label = ctk.CTkLabel(master=frame, height=10, width=100, text="ID", fg_color="gray", corner_radius=5)
    id_label.place(relx=0.05, rely=0.405)


    side_var = ctk.IntVar(value=20)
    side_entry = ctk.CTkEntry(master=frame, height=15, width=300, text_color="white",placeholder_text="side (cm)", textvariable=side_var)
    side_entry.place(relx=0.2,rely=0.5)
    side_label = ctk.CTkLabel(master=frame, height=10, width=100, text="side (cm)", fg_color="gray", corner_radius=5)
    side_label.place(relx=0.05, rely=0.505)


    border_var = ctk.IntVar(value=1)
    border_entry = ctk.CTkEntry(master=frame, height=15, width=300, text_color="white",placeholder_text="border bits", textvariable=border_var)
    border_entry.place(relx=0.2,rely=0.6)
    border_label = ctk.CTkLabel(master=frame, height=10, width=100, text="border bits", fg_color="gray", corner_radius=5)
    border_label.place(relx=0.05, rely=0.605)

    return root


def set_variables(path_var, png_var, id_var, side_var, border_var, dict_var) -> None:
    """sets variables received from the user when certain button is clicked"""
    global SIDE, ID, BORDER, PNG_NAME, PNG_PATH, TYPE
    SIDE = side_var.get() if side_var.get() > 10 else 0
    ID = id_var.get() if (id_var.get() >= 0 and id_var.get() <= 1000) else 0
    BORDER = border_var.get() if border_var.get() >= 0 else 0
    PNG_NAME = png_var.get()
    PNG_PATH = path_var.get()
    TYPE = dict_var.get()



def main() -> None:
    root = create_root()
    root.mainloop()

if __name__ == "__main__":  
    main()
    
