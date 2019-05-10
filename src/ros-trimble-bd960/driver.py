#!/usr/bin/env python

import serial
import time

class TrimbleBD960(object):
    def __init__(self, port='/dev/ttyUSB0', baudrate=38400, timeout=1):
        self._preempted = False
        self._ser = serial.Serial(port, baudrate, timeout=timeout)

        self.GGA_TAG = '$GPGGA'
        self.GST_TAG = '$GPGST'
        self.HDT_TAG = '$GPHDT'

        self.fix_status_dict = {0: 'Fix not valid',
                                1: 'Valid GPS fix',
                                2: 'Valid DGPS fix'
        }

        self.lat = 0
        self.long = 0
        self.alt = 0
        self.heading = 0
        self.covariance = 0
        self.covariance_type = 2
        self.fix_status = 0

    def __enter__(self):
        self.open()
        return self

    def __exit__(self, *args, **kwargs):
        self.close()

    def open(self):
        if not self._ser.isOpen():
            self._ser.open()

        self._ser.send_break()
        self._ser.flush()

    def close(self):
        self._ser.close()

    def readline(self):
        return self._ser.readline()

    def preempt(self):
        self._preempted = True
        self._ser.send_break()

    def read_sentence(self):
        response = self.readline().strip().split('*')
        sentence = response[0].split(',')
        if sentence:
            self.decode(sentence)
            self.publish()

    def decode_gga(self, sentence):
        self.lat = float(sentence[2])
        if sentence[3] == 'S':
            self.lat = (self.lat * -1.0)

        self.long = float(sentence[4])
        if sentence[5] == 'W':
            self.long = (self.long * -1.0)

        self.alt = float(sentence[9])
        self.fix_status = int(sentence[6])

    def decode_gst(self, sentence):
        lat_covariance = (float(sentence[6]) * float(sentence[6]))
        long_covariance = (float(sentence[7]) * float(sentence[7]))
        alt_covariance = (float(sentence[8]) * float(sentence[8]))

        self.covariance = [lat_covariance, 0.0, 0.0,
                           0.0, long_covariance, 0.0,
                           0.0, 0.0, alt_covariance]

    def decode_hdt(self, sentence):
        if sentence[1]:
            self.heading = float(sentence[1])

    def decode(self, sentence):
        tag = sentence[0]
        if tag == self.GGA_TAG:
            self.decode_gga(sentence)
        elif tag == self.GST_TAG:
            self.decode_gst(sentence)
        elif tag == self.HDT_TAG:
            self.decode_hdt(sentence)
        else:
            print('Error: Sentence code not recognized')

    def publish(self):
        print('----')
        print('Lattitude: {}'.format(self.lat))
        print('Longitude: {}'.format(self.long))
        print('Altitude: {} m'.format(self.alt))
        print('Heading: {}'.format(self.heading))
        print('Fix: {}'.format(self.fix_status_dict[self.fix_status]))

    def nmea_stream(self):
        self._preempted = False
        self._ser.send_break()
        self._ser.flush()

        while not self._preempted:
            try:
                self.read_sentence()
            except Exception as e:
                print('Error: ', str(e))

if __name__ == "__main__":
    gps = TrimbleBD960()
    gps.nmea_stream()
