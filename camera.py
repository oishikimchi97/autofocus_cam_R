import time
from threading import Thread
from streaming.ImageConvert import convert_image
import streaming.arducam_config_parser as arducam_config_parser
import streaming.ArducamSDK as ArducamSDK


class Camera:
    def __init__(self, config_file_name):
        self.running = True
        self.config_file_name = config_file_name
        self.initialize_from_file()
        ArducamSDK.Py_ArduCam_setMode(self.handle, ArducamSDK.CONTINUOUS_MODE)
        self.capture_threading = Thread(target=self.capture_thread, daemon=True)
        self.capture_threading.start()


    def initialize_from_file(self):
        # load config file
        config = arducam_config_parser.LoadConfigFile(self.config_file_name)

        camera_parameter = config.camera_param.getdict()
        Width = camera_parameter["WIDTH"]
        Height = camera_parameter["HEIGHT"]

        BitWidth = camera_parameter["BIT_WIDTH"]
        ByteLength = 1
        if BitWidth > 8 and BitWidth <= 16:
            ByteLength = 2
            save_raw = True
        FmtMode = camera_parameter["FORMAT"][0]
        self.color_mode = camera_parameter["FORMAT"][1]
        print("color mode", self.color_mode)

        I2CMode = camera_parameter["I2C_MODE"]
        I2cAddr = camera_parameter["I2C_ADDR"]
        TransLvl = camera_parameter["TRANS_LVL"]
        cfg = {"u32CameraType":0x00,
                "u32Width":Width,"u32Height":Height,
                "usbType":0,
                "u8PixelBytes":ByteLength,
                "u16Vid":0,
                "u32Size":0,
                "u8PixelBits":BitWidth,
                "u32I2cAddr":I2cAddr,
                "emI2cMode":I2CMode,
                "emImageFmtMode":FmtMode,
                "u32TransLvl":TransLvl }

        ret, self.handle, rtn_cfg = ArducamSDK.Py_ArduCam_autoopen(cfg)

        # error
        if ret != 0: print(f"open fail,rtn_val = {ret}"); return

        # Loaded camera successfully
        ArducamSDK.Py_ArduCam_writeReg_8_8(self.handle, 0x46, 3, 0x00)

        usb_version = rtn_cfg['usbType']
        for i in range(config.configs_length):
            type = config.configs[i].type
            if ((type >> 16) & 0xFF) != 0 and ((type >> 16) & 0xFF) != usb_version:
                continue
            if type & 0xFFFF == arducam_config_parser.CONFIG_TYPE_REG:
                ArducamSDK.Py_ArduCam_writeSensorReg(self.handle, config.configs[i].params[0], config.configs[i].params[1])
            elif type & 0xFFFF == arducam_config_parser.CONFIG_TYPE_DELAY:
                time.sleep(float(config.configs[i].params[0])/1000)
            elif type & 0xFFFF == arducam_config_parser.CONFIG_TYPE_VRCMD:
                ArducamSDK.Py_ArduCam_setboardConfig(self.handle, config.configs[i].params[0], config.configs[i].params[1], config.configs[i].params[2], config.configs[i].params[3], config.configs[i].params[4:config.configs[i].params_length])

        rtn_val,datas = ArducamSDK.Py_ArduCam_readUserData(self.handle, 0x400-16, 16)
        print("Serial: %c%c%c%c-%c%c%c%c-%c%c%c%c"%(datas[0],datas[1],datas[2],datas[3],datas[4],datas[5],datas[6],datas[7],datas[8],datas[9],datas[10],datas[11]))


    # occucr error if not continuous capture
    def capture_thread(self):
        rtn_val = ArducamSDK.Py_ArduCam_beginCaptureImage(self.handle)
        self.running = rtn_val == 0
        print(f"Capture began {'error' if self.running else ''}, rtn_val = {rtn_val}")
        
        while self.running:
            rtn_val = ArducamSDK.Py_ArduCam_captureImage(self.handle)
            if rtn_val > 255:
                print("Error capture image, rtn_val = ", rtn_val)
            # time.sleep(0.005)
            
        ArducamSDK.Py_ArduCam_endCaptureImage(self.handle)

    # reset chash and read fresh frame
    def shot(self):
        while ArducamSDK.Py_ArduCam_availableImage(self.handle) > 0:
            ArducamSDK.Py_ArduCam_del(self.handle)
        while ArducamSDK.Py_ArduCam_availableImage(self.handle) < 2: pass
        ArducamSDK.Py_ArduCam_del(self.handle)
        rtn_val, data, rtn_cfg = ArducamSDK.Py_ArduCam_readImage(self.handle)
        ArducamSDK.Py_ArduCam_del(self.handle)
        datasize = rtn_cfg['u32Size']
        if rtn_val != 0 or datasize == 0:
            ArducamSDK.Py_ArduCam_del(self.handle)
            print("read data fail!")
        image = convert_image(data, rtn_cfg, self.color_mode)
        return image


if __name__ == "__main__":    
    import os
    import cv2
    import datetime
    config_file_name = "OV2311_MIPI_2Lane_RAW8_8b_1600x1300_60fps.cfg"
    cam = Camera(config_file_name)
    os.makedirs('figure', exist_ok=True)
    while True:
        if input('Press Enter to capture image >> ') == 'quit': break
        cv2.imwrite(f'figure/{datetime.datetime.now().strftime("%Y%m%d_%H%M%S")}.png', cam.shot())

