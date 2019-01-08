import serial
from threading import Thread
from traceback import print_exc
import time


_rx_thread_is_running = False
_tx_thread_is_running = False
_enable_print = False
_port = serial.Serial('/dev/ttyACM0', 115200)

# control variables
v_x = 0.0             # m/s
omega = 0.0           # rad/s
odom_x = 0.0
odom_y = 0.0
odom_rot = 0.0
bumper_l = False
bumper_r = False
ultrasound = 0.0


def _reader_thread():
    counter = 0
    data_str = ""

    while _rx_thread_is_running:
        try:
            data = _port.read()
            data_char = data.decode()
            data_str += data_char
            counter += 1

            if _enable_print:
                # Print out single received characters
                sys.stdout.write(data_char)
                if data_str == ';':
                    sys.stdout.write('\n')
                sys.stdout.flush()

            if data_char == '\n':
                tmp_variable_str_list = []
                tmp_str = ''

                for char in data_str:
                    if char==',':
                        tmp_variable_str_list.append(tmp_str)
                        tmp_str = ''
                    elif char=='\r':
                        tmp_variable_str_list.append(tmp_str)
                        break
                    else:
                        tmp_str += char

                if (len(tmp_variable_str_list) == 6):
                    odom_x = float(tmp_variable_str_list[0])
                    odom_y = float(tmp_variable_str_list[1])
                    odom_rot = float(tmp_variable_str_list[2])
                    bumper_l = bool(int(tmp_variable_str_list[3]))
                    bumper_r = bool(int(tmp_variable_str_list[4]))
                    ultrasound = float(tmp_variable_str_list[5])

                    print("odom_x: %f, odom_y: %f, odom_rot: %f, bumper_l: %d, bumper_r: %d, ultrasound: %f" % (odom_x, odom_y, odom_rot, bumper_l, bumper_r, ultrasound))
                    print("\n \n")

                data_str = []

        except KeyboardInterrupt:
            _port.close()
            break
        except:
            print_exc()
            break


def _writer_thread():
    # hardcode ! this should use variables v_x, omega
    to_send = "0.100,0.100\n"

    while _tx_thread_is_running:
        _port.write(to_send.encode())
        time.sleep(0.1)


_rx_thread_is_running = True
_tx_thread_is_running = True

reader_thread = Thread(target=_reader_thread)
writer_thread = Thread(target=_writer_thread)

reader_thread.start()
writer_thread.start()

reader_thread.join()
writer_thread.join()
