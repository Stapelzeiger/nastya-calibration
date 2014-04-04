import csv

class EncData:
    def __init__(self, csv_row):
        self.timestamp = int(csv_row[0])
        self.enc0 = int(csv_row[1])
        self.enc1 = int(csv_row[2])
        self.enc2 = int(csv_row[3])
    def __repr__(self):
        return 'enc@{}'.format(self.timestamp)

class EncSpeed:
    def __init__(self, a, b):
        self.timestamp = b.timestamp
        delta_t = b.timestamp - a.timestamp
        self.w0 = (b.enc0 - a.enc0)/delta_t
        self.w1 = (b.enc1 - a.enc1)/delta_t
        self.w2 = (b.enc2 - a.enc2)/delta_t
    def __repr__(self):
        return 'encspeed@{}'.format(self.timestamp)

class IMUData:
    def __init__(self, csv_row):
        self.timestamp = int(csv_row[0])
        self.accx = float(csv_row[1])
        self.accy = float(csv_row[2])
        self.accz = float(csv_row[3])
        self.gyrox = float(csv_row[4])
        self.gyroy = float(csv_row[5])
        self.gyroz = float(csv_row[6])
    def __repr__(self):
        return 'imu@{}'.format(self.timestamp)

def read_csv(csv_file, constr):
    print(csv_file)
    data = []
    for line, row in enumerate(csv.reader(open(csv_file), delimiter=',')):
        try:
            data.append(constr(row))
        except Exception, e:
            print('invalid datapoint line: ' + str(line+1) + ' (' + str(e) + ')')
    return data

enc_data = sorted(read_csv('data/enc_test.csv', EncData), key=lambda x: x.timestamp)
imu_data = sorted(read_csv('data/imu_test.csv', IMUData), key=lambda x: x.timestamp)

enc_speed_data = []
for i in range(len(enc_data) - 1):
    enc_speed_data.append(EncSpeed(enc_data[i], enc_data[i+1]))

print(enc_data)
print(imu_data)
print(enc_speed_data)


print(sorted((enc_speed_data + imu_data), key=lambda x: x.timestamp))



'''
State variables:
pos_x
pos_y
theta
vel_x
vel_y
omega
gyro_z_null
acc_x_null
acc_y_null
imu_orientation
D0
D1
D2
R0
R1
R2

update:
acc
gyro

measurement:
encoder0-2 (wheel speed rad/s)

measurement:
position

'''

def state_transition_fn()
