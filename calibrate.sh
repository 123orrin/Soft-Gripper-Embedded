sudo chown root.gpio /dev/gpiomem
sudo chmod g+rw /dev/gpiomem
python3 ./gripper/calibration.py
