# Install instructions for [Python](#python) and OpenCV

(and a quick reminder for me
command pa que se vea la camara en el raspberry pi: `libcamera-hello -t 0`
<br/>command pa activar el venv en raspberry pi: ```source ~/path/to/venv/bin/activate```
<br/>pa apagar el venv: ```deactivate```)


# Download Python and OpenCV (on your computer, if you want)
## Python
For using the code, you need to have Python on your machine. I used Python 3.11.[something] on a Raspberry Pi (using Raspberry Pi OS, which is Unix). To check if Python is installed, open a terminal, command, or PowerShell window and type `python --version`, `python3 --version`, or `py --version` and hit enter. If it's not installed, you can go to [Python's webpage](https://www.python.org/downloads/) and download it from there.

## Clone repo
After Python is installed, clone this repository (or before, it doesnt matter which one you do first). You can either navigate to the folder on your computer where you want the project to be located and run `git clone https://github.com/natanael-2012/RasCas-Hielo.git`, or be in any directory and run `git clone https://github.com/natanael-2012/RasCas-Hielo.git C:\path\to\directory\`. When this project is cloned, a folder called RasCas-Hielo should appear. 

## Create  and activate a venv (for Windows, idk for Mac)
To download all modules and dependencies, you will need to create a **virtual environment**. You could do it without a virtual environment, but its not recommendable. To create a virtual environment: 
  1. first go to the folder where you will be working, that is, the one we just cloned (RasCas-Hielo): `cd C:\path\to\your\folder\RasCas-Hielo`
  2. After you are inside your working folder, run `python -m venv name_of_your_venv`. 
  3. Alternatively, you can ignore steps 1 and 2 and simply run `python -m venv C:\path\to\folder\RasCas-Hielo\name_of_your_venv`
  4. wait a bit
     
A folder with the `name_of_your_venv` should be created *inside* the `RasCas-Hielo` folder. There are several ways to activate the venv, but I always activate it like this:
  1. Go to your working folder. I assume you are already there, but if not, then run `cd C:\path\to\folder\RasCas-Hielo`
  2. Run `name_of_your_venv/Scripts/Activate`
  3. If everything worked correctly, you should see at the beginning of the line "`(name_of_your_venv)`". This usually appears in green.

### Deactivating your venv
When you are finished working with your virtual environment, simply run `deactivate`. 

### Reminder
If your venv is not called `venv` (and located inside the RasCas-Hielo folder, add it to the `.gitignore` file pls.

## Download OpenCV
With your **venv activated**. Type the following, using either `python`, `python3`, or `py` that you used before. 
- for Windows `py -m pip install --upgrade pip` and then `py -m pip --version`

Then, run `pip install opencv-contrib-python`. It should download and install. 

Congrats! You have downloaded OpenCV in your PC. 





## OpenCV
Make sure you are using a 64-bit OS, since OpenCV won't work on 32-bits.  
You can install OpenCV with pip install. Make sure the venv is activated, and run this on the terminal: `pip install opencv-contrib-python`


## Tutorial que estoy siguiendo por ahora
https://pyimagesearch.com/2020/12/28/determining-aruco-marker-type-with-opencv-and-python/ 

https://pyimagesearch.com/2020/12/21/detecting-aruco-markers-with-opencv-and-python/

