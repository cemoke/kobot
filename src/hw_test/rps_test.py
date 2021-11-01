import sys
import struct
import time
from smbus2 import SMBus, i2c_msg


class TOF:
    """
    Each object corresponds to a VL53L1X sensor on the RPS board
    """

    def __init__(self, bus, sensor_id):
        self.bus = bus
        self.sensor_id = sensor_id
        self.distance = 0
        self.is_robot = False
        self.status = 0
        self.ambient = 0
        self.signal = 0
        self.s_r = 0
        self.num_SPAD = 0

    def __str__(self):
        return "ID : {}, Distance : {}, isRobot : {}, ambient : {}, signal : {}, status : {}, s_r : {}".format(self.sensor_id,
                                                             self.distance,
                                                             self.is_robot,
                                                             self.ambient,
                                                             self.signal,
                                                             self.status,
                                                             self.s_r)

    def parse_result_data(self, block):
        """
        Result data type is read from MCU
        /**
        *  @brief defines packed reading results type
        */
        typedef struct {
        uint8_t Status;		/*!< ResultStatus */
        uint16_t Distance;	/*!< ResultDistance */
        uint16_t Ambient;	/*!< ResultAmbient */
        uint16_t SigPerSPAD;/*!< ResultSignalPerSPAD */
        uint16_t NumSPADs;	/*!< ResultNumSPADs */
        } VL53L1X_Result_t;
        """
        string = ""
        # convert array of ints to a string
        for integer_val in block:
            string += chr(integer_val)
        self.distance = struct.unpack("=H", string[0:2][::-1])[0]
        if self.distance > 30000:
            self.distance =  self.distance - 2**15
    def parse_rb_result_data(self, block):
        """
        Result data type is read from MCU
        /**
        *  @brief defines packed reading results type
        */
        typedef struct {
        uint8_t Status;		/*!< ResultStatus */
        uint16_t Distance;	/*!< ResultDistance */
        uint16_t Ambient;	/*!< ResultAmbient */
        uint16_t SigPerSPAD;/*!< ResultSignalPerSPAD */
        uint16_t NumSPADs;	/*!< ResultNumSPADs */
        } VL53L1X_Result_t;
        """
        # print('{:08b}'.format(block[0]))
        # print('{:08b}'.format(block[0]))

        self.is_robot = False
        MSB =block[0]
        if (MSB & 0x10):
            MSB = MSB & (~0x10)
            self.is_robot = True
        block[0] = MSB
        distance = struct.unpack("=H", chr(block[1])+chr(block[0]))[0]
        if distance > 5000:
            # print("o")
            self.distance = -(2**15-distance)
        else:
            self.distance = distance
    def parse_ambient_result_data(self, block):
        ambient = block
        if ambient > 127:
            self.ambient = -(2**7-ambient)
        else:
            self.ambient = ambient
    def parse_signal_result_data(self, block):
        ambient = block
        if ambient > 127:
            self.signal = -(2**7-ambient)
        else:
            self.signal = ambient
        try:
            self.s_r = float(self.ambient)/float(self.signal)
        except ZeroDivisionError:
            self.s_r = 1
    def parse_status_result_data(self, block):
        ambient = block
        if ambient > 127:
            self.status = -(2**7-ambient)
        else:
            self.status = ambient


    def read_single_result(self):
        self.bus.write_byte(self.addr, self.regr)
        block = self.bus.read_i2c_block_data(self.addr, 252, 9)
        self.parse_result_data(block)


class RPS:
    """
    Corresponds to the Relative Positioning System
    """
    # // When looking from IC direction left inner sensor is J1
    # // top left of J1 we have J12 outer ring IDs increases
    # // in CCW whereas inner ring IDs increases in CW
    # // first tof shutPin->0 second shutPin-> 1 etc.
    # // Shut pin Mappings
    # // {J1:PA8,J2:PA15,J3:PB14,J4:PB4,J5:PB12,J6:PC14,
    # // J7:PA3,J8:PA4,J9:PB5,J10:PB13,J11:PB3,J12:PB15}

    addr = 10

    def __init__(self):
        try:
            self.bus = SMBus(1)
            time.sleep(1)
            # if self.read_err():
            #     print("Error on RPS")
            #     self.read_err_sensor_id()
            #     print("Error on TOF {}".format(self.err_sensor_id))
            #     sys.exit()
        except IOError:
            print("IO Error on I2C")
            time.sleep(1)
            sys.exit()
        self.tof_arr = []
        self.read_num_sensors()
        print("{} TOF sensors found on RPS".format(self.num_sensors))
        self.construct_sensor_array()

    def write_ambient_thresh(self,val):
        self.bus.write_byte_data(self.addr, 0, val)
    def read_num_sensors(self):
        self.bus.write_byte(self.addr, 249)
        num_sensors = self.bus.read_byte_data(self.addr, 249)
        print(num_sensors)
        self.bus.write_byte(self.addr, 248)
        num_zones = self.bus.read_byte_data(self.addr, 248)
        print(num_zones)
        self.num_sensors = num_sensors*num_zones
        self.num_sensors = 12

    def read_err(self):
        self.bus.write_byte(self.addr, 254)
        val = self.bus.read_byte_data(self.addr, 252)
        try:
            self.error = struct.unpack("=B", chr(val-128))[0]
            print(self.error)
        except ValueError:
            self.error = 0
            print("num  sensors problem")
        # print("Error : ", self.error)

    def read_err_sensor_id(self):
        self.bus.write_byte(self.addr, 253)
        block = self.bus.read_byte_data(self.addr, 252)
        self.err_sensor_id = struct.unpack("=B", chr(val-128))[0]
        # print("Err Sensor Id : ", self.err_sensor_id)

    def construct_sensor_array(self):
        for i in range(self.num_sensors):
            self.tof_arr.append(TOF(self.bus, i+1))

    def read_results(self):
        self.bus.write_byte(self.addr, 252)
        block = self.bus.read_i2c_block_data(
            self.addr, 252, self.num_sensors*2)
        self.parse_results_data(block)
    def read_rb_results(self):
        # self.bus.write_byte(self.addr, 255)
        # # time.sleep(0.1)
        # # write = i2c_msg.write(self.addr, [255])
        # # read = i2c_msg.read(self.addr, 24)
        # block1 = self.bus.read_i2c_block_data(
        #     self.addr, 255, self.num_sensors)
        # self.bus.write_byte(self.addr, 254)
        # block2 = self.bus.read_i2c_block_data(
        #     self.addr, 254, self.num_sensors)
        # self.parse_results_data(block1+block2)
        self.bus.write_byte(self.addr, 253)
        block = self.bus.read_i2c_block_data(
            self.addr, 253, 24)
        self.parse_results_data(block)
    def read_ambient_results(self):
        self.bus.write_byte(self.addr, 252)
        block = self.bus.read_i2c_block_data(
            self.addr, 252, self.num_sensors)
        self.parse_ambient_data(block)
    def read_signal_results(self):
        self.bus.write_byte(self.addr, 251)
        block = self.bus.read_i2c_block_data(
            self.addr, 251, self.num_sensors)
        self.parse_signal_data(block)
    def read_status_results(self):
        self.bus.write_byte(self.addr, 250)
        block = self.bus.read_i2c_block_data(
            self.addr, 250, self.num_sensors)
        self.parse_status_data(block)
    def parse_results_data(self, block):
        for i, tof in enumerate(self.tof_arr):
            tof.parse_rb_result_data(block[2*i:2*(i+1)])
    def parse_ambient_data(self, block):
        for i, tof in enumerate(self.tof_arr):
            tof.parse_ambient_result_data(block[i])
    def parse_signal_data(self, block):
        for i, tof in enumerate(self.tof_arr):
            tof.parse_signal_result_data(block[i])
    def parse_status_data(self, block):
        for i, tof in enumerate(self.tof_arr):
            tof.parse_status_result_data(block[i])
    def __str__(self):
        out_str = ""
        for i in range(self.num_sensors):
            out_str += str(self.tof_arr[i]) + '\n'
        return out_str


if __name__ == "__main__":
    rps = RPS()
    i = 60
    rps.write_ambient_thresh(i)
    while True:
        rps.read_rb_results()
        # time.sleep(0.005)
        # rps.read_ambient_results()
        # time.sleep(0.005)
        # rps.read_signal_results()
        # time.sleep(0.005)
        # rps.read_status_results()
        time.sleep(0.05)
        print(rps)
